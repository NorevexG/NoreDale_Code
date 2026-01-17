// UnitMovementComponent.cpp
//To Do befor calling this finished:
// - remove debug code from avoidance check

#include "UnitMovementComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "NavigationSystem.h"
#include "NavigationPath.h"
#include "DrawDebugHelpers.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "Engine/EngineTypes.h"

UUnitMovementComponent::UUnitMovementComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = true;
    PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UUnitMovementComponent::BeginPlay()
{
    Super::BeginPlay();
    RefreshMovementData();
}

void UUnitMovementComponent::RefreshMovementData()
{
    CharacterOwner = Cast<ACharacter>(GetOwner());
    if (!CharacterOwner)
        return;

    AvoidanceSide = (FMath::RandBool() ? 1 : -1);

    CapComp = CharacterOwner->GetCapsuleComponent();
    if (CapComp) { //does character have capsule component
        UnitRadius = CapComp->GetScaledCapsuleRadius(); //set my radius to capsule radius
    }
    else if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(CharacterOwner->GetRootComponent())) //if other root component has bounds (size)
    {
        FVector E = RootPrim->Bounds.BoxExtent; //size of object
        UnitRadius = FMath::Max(E.X, E.Y); //choose larger value
    }
    RotationSizeScale = UnitRadius / FMath::Max(RotationModifier, 1.f);

    MoveComp = CharacterOwner->GetCharacterMovement();
    if (!MoveComp) return;
}

TArray<FVector> UUnitMovementComponent::MoveToLocation(const FVector& TargetLocation, bool bUseNav /*= true*/)
{
    StopMovement();

    UWorld* World = GetWorld();
    UNavigationSystemV1* NavSys = World ? FNavigationSystem::GetCurrent<UNavigationSystemV1>(World) : nullptr;

    FVector UseTarget = TargetLocation;

    if (bUseNav && bUseNavMesh && NavSys)
    {
        FNavLocation Projected;
        if (NavSys->ProjectPointToNavigation(TargetLocation, Projected, FVector(200.f, 200.f, 500.f)))
        {
            UseTarget = Projected.Location;

            if (BuildPathToLocation(UseTarget))
            {
                bMoving = true;
                CurrentPathIndex = 0;
                return InternalPathPoints;
            }
        }
    }

    RefreshMovementData();
    InternalPathPoints.Empty();
    InternalPathPoints.Add(UseTarget);
    CurrentPathIndex = 0;
    bMoving = true;

    return InternalPathPoints;
}

TArray<FVector> UUnitMovementComponent::MoveToActor(AActor* TargetActor)
{
    TArray<FVector> Empty;
    if (!TargetActor) return Empty;

    MoveTargetActor = TargetActor;
    RefreshMovementData();

    return MoveToLocation(TargetActor->GetActorLocation(), bUseNavMesh);
}

void UUnitMovementComponent::SetPath(const TArray<FVector>& PathPoints, bool bStartMoving /*= true*/)
{
    InternalPathPoints = PathPoints;
    CurrentPathIndex = 0;
    bMoving = bStartMoving && InternalPathPoints.Num() > 0;
    MoveTargetActor = nullptr;
}

void UUnitMovementComponent::StopMovement()
{
    bMoving = false;
    InternalPathPoints.Empty();
    CurrentPathIndex = 0;
    MoveTargetActor = nullptr;
}

bool UUnitMovementComponent::BuildPathToLocation(const FVector& TargetLocation)
{
    UWorld* World = GetWorld();
    if (!World) return false;

    UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
    if (!NavSys) return false;

    AActor* Owner = GetOwner();
    if (!Owner) return false;

    UNavigationPath* NavPath = NavSys->FindPathToLocationSynchronously(
        World,
        Owner->GetActorLocation(),
        TargetLocation,
        Owner
    );

    if (!NavPath || NavPath->PathPoints.Num() == 0 || NavPath->IsPartial())
        return false;

    InternalPathPoints = NavPath->PathPoints;
    CurrentPathIndex = 0;
    return true;
}

bool UUnitMovementComponent::TickMoveAlongPath(float DeltaTime)
{
    if (!bMoving || InternalPathPoints.Num() == 0 || !CharacterOwner)
        return true;

    if (MoveTargetActor.IsValid())
        InternalPathPoints.Last() = MoveTargetActor->GetActorLocation();

    FVector Pos = CharacterOwner->GetActorLocation();

    while (CurrentPathIndex < InternalPathPoints.Num())
    {
        FVector Next = InternalPathPoints[CurrentPathIndex];
        FVector ToNext = Next - Pos;

        float Dist2D = FVector(ToNext.X, ToNext.Y, 0.f).Size();

        if (Dist2D > UnitRadius * AcceptanceModifier)
            break;

        CurrentPathIndex++;
    }
    
    if (CurrentPathIndex >= InternalPathPoints.Num())
    {
        if (!ClippingFix(Pos))
        {
            StopMovement();
            return true;
        }
        else
        {
            CurrentPathIndex = InternalPathPoints.Num() - 1;
        }
    }

    FVector Next = InternalPathPoints[CurrentPathIndex];
    FVector ToNext = Next - Pos;

    FVector DesiredDir = FVector(ToNext.X, ToNext.Y, 0.f).GetSafeNormal();
    if (DesiredDir.IsNearlyZero())
        return false;

    CurrentGoal = Next;

    bool bGoalAdjusted = AvoidanceCheck(DesiredDir, Pos, DeltaTime);
    if (bGoalAdjusted)
        return false;

    if (MoveComp)
    {
        if (MoveComp->MovementMode == MOVE_None)
            MoveComp->SetMovementMode(MOVE_Walking);

        MoveComp->MaxWalkSpeed = MoveSpeed;
    }

    CharacterOwner->AddMovementInput(DesiredDir, 1.f);
    SmoothFaceDirection(DesiredDir, DeltaTime);

    return false;
}

//Reworked Version 1
bool UUnitMovementComponent::AvoidanceCheck(FVector& DesiredDir, const FVector& MyPos, float DeltaTime)
{
    bool bIsClipping = false;
    bool bGoalAdjusted = false;
    if (!CharacterOwner)
        return bGoalAdjusted;

    // --- setup ---
    FVector ToGoal = CurrentGoal - MyPos;
    FVector ToGoal2D(ToGoal.X, ToGoal.Y, 0.f);
    float DistToGoal = ToGoal2D.Size();
    float Range = FMath::Max(50.f, AvoidanceCheckDistance);
    if (DistToGoal < KINDA_SMALL_NUMBER) return bGoalAdjusted;

    //Debuging Variables
    //FVector ForwardEnd = MyPos + DesiredDir * Range;
    FVector ForwardEnd = MyPos + FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal() * Range;
    bool bDetectedInFront = false;
    bool bBlockingPath = false;
    FColor ForwardColor = FColor::Green;

    // Overlap query to find nearby pawns/physics bodies
    TArray<FOverlapResult> Overlaps; //array of type overlap results
    FCollisionObjectQueryParams ObjQuery; //stores the type of objects to look for, each one is below
    ObjQuery.AddObjectTypesToQuery(ECC_Pawn);
    ObjQuery.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams Params(SCENE_QUERY_STAT(AvoidanceOverlap), true, CharacterOwner);

    bool bAny = GetWorld()->OverlapMultiByObjectType( //bool to tell if hit anything
        Overlaps,           //adding each element hit to array
        MyPos,
        FQuat::Identity,    //used to say dont rotate, aligned with world axis
        ObjQuery,
        FCollisionShape::MakeSphere(Range),
        Params
    );

    TArray<AActor*> BlockingActors;

    if (bAny && Overlaps.Num() > 0) //if i did hit something and it was valid
    {
        FVector Forward2D = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
        if (!Forward2D.IsNearlyZero())
        {
            // My velocities (2D)
            FVector MyVel2D = CharacterOwner->GetVelocity();
            MyVel2D.Z = 0.f;
            if (MyVel2D.IsNearlyZero())
                MyVel2D = Forward2D * MoveSpeed;

            float MySpeed2D = MyVel2D.Size();
            if (MySpeed2D < 1.f) MySpeed2D = FMath::Max(1.f, MoveSpeed);
            float tMax = Range / MySpeed2D;          // e.g., 600/100 = 6s
            tMax = FMath::Clamp(tMax, 0.f, 8.f);     // safety clamp

            for (const FOverlapResult& R : Overlaps)
            {
                AActor* Other = R.GetActor();
                if (!Other || Other == CharacterOwner) continue; //if other is nullptr or us skip this for loop turn
                if (MoveTargetActor.IsValid() && Other == MoveTargetActor.Get()) continue; //if we are following an actor and other is equal to it skip this for loop turn

                FVector OtherLoc = Other->GetActorLocation();
                FVector ToOther2D(OtherLoc.X - MyPos.X, OtherLoc.Y - MyPos.Y, 0.f); //diffrence between units
                float DistToOther = ToOther2D.Size();
                if (DistToOther < KINDA_SMALL_NUMBER) continue; //might need tweaking not sure

                // --- dynamic radii ---
                float OtherRadius = SetOtherRadius(Other);

                float PathCorridor = UnitRadius + OtherRadius + ExtraBuffer;

                // Other velocities (2D)
                FVector OtherVel2D = Other->GetVelocity();
                OtherVel2D.Z = 0.f;
                if (OtherVel2D.IsNearlyZero())
                {
                    if (UUnitMovementComponent* OUnitComp = Other->FindComponentByClass<UUnitMovementComponent>())
                    {
                        OtherVel2D = (-ToOther2D.GetSafeNormal()) * OUnitComp->MoveSpeed;
                    }
                    else if (ACharacter* OC = Cast<ACharacter>(Other))
                    {
                        if (UCharacterMovementComponent* OMoveComp = OC->GetCharacterMovement())
                            OtherVel2D = (-ToOther2D.GetSafeNormal()) * OMoveComp->MaxWalkSpeed;
                    }
                }

                FVector RelVel2D = OtherVel2D - MyVel2D;
                float RelVelSq = RelVel2D.SizeSquared();
                float tClosest = 0.f;
                FVector RelAtClosest = ToOther2D;

                if (RelVelSq > 25.f)
                {
                    tClosest = -FVector::DotProduct(ToOther2D, RelVel2D) / RelVelSq;
                    tClosest = FMath::Clamp(tClosest, 0.f, tMax);
                    RelAtClosest = ToOther2D + RelVel2D * tClosest;
                }

                //lambda function is actully bool InDynamicCorridor(const FVector& Rel)
                auto InDynamicCorridor = [&](const FVector& Rel) -> bool
                {
                    float Along = FVector::DotProduct(Rel, Forward2D);
                    if (Along <= 0.f || Along > Range)
                        return false;

                    FVector ClosestRel = Forward2D * Along;
                    float PerpDist = FVector(Rel.X - ClosestRel.X, Rel.Y - ClosestRel.Y, 0.f).Size();
                    return PerpDist < PathCorridor;
                };

                bool bInCorridorNow = InDynamicCorridor(ToOther2D);
                bool bInCorridorSoon = InDynamicCorridor(RelAtClosest);
                // X-crossing fix: collide soon even if not "ahead"
                bool bWillCollideSoon = (RelAtClosest.Size() < PathCorridor);

                bool bBlockingDynamic = (bInCorridorNow || bInCorridorSoon || bWillCollideSoon);

                if (bBlockingDynamic)
                {
                    BlockingActors.AddUnique(Other);
                }

                bDetectedInFront = true; //DEBUG
            } // for overlaps
        }

        if (BlockingActors.Num() > 0)
        {
            if (CurrentPathIndex >= InternalPathPoints.Num() - 1)
            {
                bGoalAdjusted = GoalAdjustment();
                if (bGoalAdjusted)
                {
                    return bGoalAdjusted; //ajusted
                }
            }
            bBlockingPath = true; //DEBUG
            ApplyAvoidance(DesiredDir, MyPos, BlockingActors);
            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage((uint64)((PTRINT)this), 187, FColor::Red, FString::Printf(TEXT("Avoid")));
            }
        }
    } // bAny

    /*-----------------------------------------DEBUG----------------------------------------------------*/
    // Clipping visualizer: check true capsule overlap (overlap using our capsule)
    if (CapComp)
    {
        FVector CapCenter = CapComp->GetComponentLocation();
        float CapRadius = CapComp->GetScaledCapsuleRadius();
        float CapHalfHeight = CapComp->GetScaledCapsuleHalfHeight();

        // Overlap with capsule shape directly to find true clipping with pawns
        TArray<FOverlapResult> ClippingHits;
        FCollisionQueryParams ClipParams(SCENE_QUERY_STAT(ClippingOverlap), true, CharacterOwner);
        bool bClipAny = GetWorld()->OverlapMultiByObjectType(
            ClippingHits,
            CapCenter,
            FQuat::Identity,
            FCollisionObjectQueryParams(ECC_Pawn),
            FCollisionShape::MakeCapsule(CapRadius, CapHalfHeight),
            ClipParams
        );

        if (bClipAny)
        {
            for (const FOverlapResult& CH : ClippingHits)
            {
                AActor* Other = CH.GetActor();
                if (!Other || Other == CharacterOwner) continue;

                if (bIsClipping)
                {
                    // draw hard red capsule and line indicating clipping
                    DrawDebugCapsule(GetWorld(), CapCenter, CapHalfHeight, CapRadius, FQuat::Identity, FColor::Purple, false, 0.05f, 0, 2.f);
                    DrawDebugLine(GetWorld(), MyPos, Other->GetActorLocation(), FColor::Purple, false, 0.05f, 0, 3.f);
                }
                else
                {
                    // draw hard red capsule and line indicating clipping
                    DrawDebugCapsule(GetWorld(), CapCenter, CapHalfHeight, CapRadius, FQuat::Identity, FColor::Red, false, 0.05f, 0, 2.f);
                    DrawDebugLine(GetWorld(), MyPos, Other->GetActorLocation(), FColor::Red, false, 0.05f, 0, 3.f);
                }
            }
        }
    }

    // Decide forward color: red if any blocking, blue if any in-front (but not blocking), else green
    if (bBlockingPath) ForwardColor = FColor::Red;
    else if (bDetectedInFront) ForwardColor = FColor::Blue;
    else ForwardColor = FColor::Green;

    if (bDebugAvoidance)
    {
        DrawDebugLine(GetWorld(), MyPos, ForwardEnd, ForwardColor, false, 0.05f, 0, 4.f);

        // If clipping, draw a thicker persistent overlay to make it obvious
        if (bIsClipping)
        {
            // Draw a small world-space text at actor location (requires engine with debug text support)
#if WITH_EDITOR
            if (GEngine)
            {
                GEngine->AddOnScreenDebugMessage((uint64)((PTRINT)this), 0, FColor::Red, FString::Printf(TEXT("CLIPPING")));
            }
#endif
        }
    }
    /*-----------------------------------------DEBUG----------------------------------------------------*/
    return bGoalAdjusted;
}

//ApplyAvoidance
// Version 5 — Predictive + near-field separation + stable velocity + smoothing
void UUnitMovementComponent::ApplyAvoidance(FVector& DesiredDir, const FVector& MyPos, TArray<AActor*> NearActors)
{
    // --- 2D only ---
    const FVector ForwardDir = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
    if (ForwardDir.IsNearlyZero() || !MoveComp)
        return;

    // Use Actor velocity (more consistent than CharacterMovement->Velocity timing)
    FVector MyVel2D = GetOwner()->GetVelocity();
    MyVel2D.Z = 0.f;
    if (MyVel2D.IsNearlyZero())
        MyVel2D = ForwardDir * MoveComp->MaxWalkSpeed;

    // --- SPEED NORMALIZATION (keep your feel) ---
    const float Speed = MoveComp->MaxWalkSpeed;
    float SpeedFactor = FMath::Clamp(Speed / 600.f, 0.5f, 2.0f);

    // Earlier prediction at higher speeds (seconds)
    const float Horizon = FMath::Clamp(0.25f + 0.20f * SpeedFactor, 0.25f, 0.55f);

    float TotalSideWeight = 0.f;
    FVector AccumulatedSideDir = FVector::ZeroVector;

    for (AActor* Other : NearActors)
    {
        const FVector OtherPos = Other->GetActorLocation();
        const FVector RelPos2D = FVector(OtherPos.X - MyPos.X, OtherPos.Y - MyPos.Y, 0.f);

        const float DistNow = RelPos2D.Size();
        if (DistNow < KINDA_SMALL_NUMBER)
            continue;

        const FVector DirToOtherNow = RelPos2D / DistNow;

        // Front-hemisphere filter
        const float ForwardDot = FVector::DotProduct(ForwardDir, DirToOtherNow);
        if (ForwardDot < -0.1f)
            continue;

        const float OtherRadius = SetOtherRadius(Other);
        const float CombinedRadius = UnitRadius + OtherRadius + ExtraBuffer;

        // --- Stable other velocity ---
        FVector OtherVel2D = Other->GetVelocity();
        OtherVel2D.Z = 0.f;

        // If other velocity is near zero, assume it is moving roughly forward at its max speed if possible
        if (OtherVel2D.IsNearlyZero())
        {
            if (ACharacter* OtherChar = Cast<ACharacter>(Other))
            {
                if (UCharacterMovementComponent* OtherMove = OtherChar->GetCharacterMovement())
                {
                    const float OtherSpeed = OtherMove->MaxWalkSpeed;
                    // Approx direction: if moving, velocity would exist; so use "toward me" as worst-case
                    OtherVel2D = (-DirToOtherNow) * OtherSpeed;
                }
            }
        }

        const FVector RelVel2D = OtherVel2D - MyVel2D;
        const float RelVelSq = RelVel2D.SizeSquared();

        // Closest approach within horizon
        float t = 0.f;
        FVector ClosestVec = RelPos2D;

        if (RelVelSq > 25.f)
        {
            t = -FVector::DotProduct(RelPos2D, RelVel2D) / RelVelSq;
            t = FMath::Clamp(t, 0.f, Horizon);
            ClosestVec = RelPos2D + RelVel2D * t;
        }

        const float ClosestDist = ClosestVec.Size();

        // --- LOOKAHEAD (key) ---
        // Make this *distance-based* so it still works at any speed
        const float LookAhead = (CombinedRadius * (2.6f * SpeedFactor)) + (Speed * Horizon);

        // Use the *minimum* of current and predicted edge distance (prevents “late wake-up”)
        const float EdgeNow = DistNow - CombinedRadius;
        const float EdgePred = ClosestDist - CombinedRadius;
        const float EdgeDist = FMath::Min(EdgeNow, EdgePred);

        if (EdgeDist > LookAhead)
            continue;

        // --- proximity (old feel) ---
        float Proximity = 1.f - FMath::Clamp(EdgeDist / LookAhead, 0.f, 1.f);
        Proximity = FMath::Pow(Proximity, 1.35f);

        // --- near-field separation (prevents sticking) ---
        // If we are very close, force stronger lateral slide using CURRENT direction.
        const bool bVeryClose = (EdgeNow < CombinedRadius * 0.10f);

        FVector DirForSide = bVeryClose ? DirToOtherNow : ClosestVec.GetSafeNormal2D();
        if (DirForSide.IsNearlyZero())
            DirForSide = DirToOtherNow;

        float SideSign = FVector::CrossProduct(ForwardDir, DirForSide).Z;

        // Perfectly centered / ambiguous -> use bias
        if (FMath::Abs(SideSign) < 0.03f)
            SideSign = (float)AvoidanceSide;

        FVector SideDir =
            (SideSign >= 0.f)
            ? FVector(ForwardDir.Y, -ForwardDir.X, 0.f)
            : FVector(-ForwardDir.Y, ForwardDir.X, 0.f);

        SideDir.Normalize();

        float EmergencyBoost = FMath::GetMappedRangeValueClamped(
            FVector2D(0.f, CombinedRadius * 0.6f),
            FVector2D(1.55f, 1.0f),
            EdgeDist
        );

        float SideWeight = Proximity * EmergencyBoost;

        // Extra shove when extremely close (stuck fix)
        if (bVeryClose)
            SideWeight = FMath::Max(SideWeight, 0.85f);

        AccumulatedSideDir += SideDir * SideWeight;
        TotalSideWeight += SideWeight;
    }

    if (TotalSideWeight <= KINDA_SMALL_NUMBER)
        return;

    const FVector FinalSideDir = AccumulatedSideDir.GetSafeNormal();

    float ClampedSideWeight = FMath::Clamp(TotalSideWeight, 0.f, 1.f);
    float ForwardWeight = 1.f - ClampedSideWeight;

    FVector NewDir = (ForwardDir * ForwardWeight) + (FinalSideDir * ClampedSideWeight);
    FVector FinalDir = NewDir.GetSafeNormal();

    // Forward-only motion
    if (FVector::DotProduct(FinalDir, ForwardDir) <= 0.f)
        return;

    // --- smoothing (removes clunky “shove”) ---
    // Keeps constant speed; only smooths direction changes.
    if (!PrevAvoidDir2D.IsNearlyZero())
    {
        FinalDir = (PrevAvoidDir2D * 0.70f + FinalDir * 0.30f).GetSafeNormal();
    }
    PrevAvoidDir2D = FinalDir;

    DesiredDir = FinalDir;
}

// Old Avoidance
void UUnitMovementComponent::OldApplyAvoidance(FVector& DesiredDir, const FVector& MyPos, const FVector& OtherPos, float OtherRadius)
{
    // --- 2D only ---
    FVector ForwardDir = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
    if (ForwardDir.IsNearlyZero())
        return;

    FVector ToOther = FVector(OtherPos.X - MyPos.X, OtherPos.Y - MyPos.Y, 0.f);
    float DistToOther = ToOther.Size();
    if (DistToOther < KINDA_SMALL_NUMBER)
        return;

    FVector DirToOther = ToOther / DistToOther;

    // Only react if roughly in front
    float ForwardDot = FVector::DotProduct(ForwardDir, DirToOther);
    if (ForwardDot < -0.1f)
        return;

    // --- SPEED NORMALIZATION ---
    float Speed = MoveComp ? MoveComp->MaxWalkSpeed : 600.f;
    float SpeedFactor = Speed / 600.f; // 600 == baseline
    SpeedFactor = FMath::Clamp(SpeedFactor, 0.5f, 2.0f);

    // --- RADII ---
    float CombinedRadius = UnitRadius + OtherRadius + ExtraBuffer;

    // --- LOOKAHEAD (speed-scaled) ---
    float LookAhead = CombinedRadius * (2.5f * SpeedFactor);

    // --- PROXIMITY (edge-to-edge) ---
    float EdgeDist = DistToOther - CombinedRadius;

    // Normalize proximity: 1 = very close, 0 = far
    float Proximity = 1.f - FMath::Clamp(EdgeDist / LookAhead, 0.f, 1.f);

    // Sharper response when close, softer earlier
    Proximity = FMath::Pow(Proximity, 1.35f);

    // --- SIDE DIRECTION ---
    float SideSign = FVector::CrossProduct(ForwardDir, DirToOther).Z;
    FVector SideDir =
        (SideSign >= 0.f)
        ? FVector(ForwardDir.Y, -ForwardDir.X, 0.f)
        : FVector(-ForwardDir.Y, ForwardDir.X, 0.f);

    SideDir.Normalize();

    // --- EMERGENCY BOOST (very close) ---
    float EmergencyBoost = FMath::GetMappedRangeValueClamped(
        FVector2D(0.f, CombinedRadius * 0.5f),
        FVector2D(1.4f, 1.0f),
        EdgeDist
    );

    // --- FINAL STEERING WEIGHT ---
    float SideWeight = Proximity * EmergencyBoost;
    float ForwardWeight = 1.f - SideWeight;

    FVector NewDir =
        (ForwardDir * ForwardWeight) +
        (SideDir * SideWeight);

    DesiredDir = NewDir.GetSafeNormal();
}

//GoalAdjustment
bool UUnitMovementComponent::GoalAdjustment()
{
    if (InternalPathPoints.Num() == 0)
        return false;

    FVector CurrentFinalGoal = InternalPathPoints.Last();

    float Range = FMath::Max(50.f, AvoidanceCheckDistance);
    TArray<FOverlapResult> NearObjects;
    FCollisionObjectQueryParams NearObjQuery; //stores the type of objects to look for, each one is below
    NearObjQuery.AddObjectTypesToQuery(ECC_Pawn);
    NearObjQuery.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams NearObjParams(SCENE_QUERY_STAT(ClippingOverlap), true, CharacterOwner);

    bool bNearObjAny = GetWorld()->OverlapMultiByObjectType( //bool to tell if hit anything
        NearObjects,           //adding each element hit to array
        CurrentFinalGoal,
        FQuat::Identity,    //used to say dont rotate, aligned with world axis
        NearObjQuery,
        FCollisionShape::MakeSphere(Range),
        NearObjParams
    );

    if (bNearObjAny)
    {
        for (const FOverlapResult& O : NearObjects)
        {
            AActor* Other = O.GetActor();
            if (!Other || Other == CharacterOwner) continue;

            FVector OtherLoc = Other->GetActorLocation();

            // --- dynamic radii ---
            float OtherRadius = SetOtherRadius(Other);

            const float MinSafeDistance = UnitRadius + OtherRadius + ExtraBuffer;
            const float DistanceSq = FVector::DistSquared2D(CurrentFinalGoal, OtherLoc);

            if (DistanceSq < FMath::Square(MinSafeDistance))
            {
                return ComputeSafestPoint(NearObjects);
            }
        }
    }
    return false;
}

//ClippingFix
bool UUnitMovementComponent::ClippingFix(const FVector& MyPos)
{
    float Range = FMath::Max(50.f, AvoidanceCheckDistance);
    TArray<FOverlapResult> ClippingHits;
    FCollisionObjectQueryParams ClipQuery; //stores the type of objects to look for, each one is below
    ClipQuery.AddObjectTypesToQuery(ECC_Pawn);
    ClipQuery.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams ClipParams(SCENE_QUERY_STAT(ClippingOverlap), true, CharacterOwner);

    bool bClipAny = GetWorld()->OverlapMultiByObjectType( //bool to tell if hit anything
        ClippingHits,           //adding each element hit to array
        MyPos,
        FQuat::Identity,    //used to say dont rotate, aligned with world axis
        ClipQuery,
        FCollisionShape::MakeSphere(Range),
        ClipParams
    );

    if (bClipAny)
    {
        for (const FOverlapResult& CH : ClippingHits)
        {
            AActor* Other = CH.GetActor();
            if (!Other || Other == CharacterOwner) continue;

            FVector OtherLoc = Other->GetActorLocation();

            // --- dynamic radii ---
            float OtherRadius = SetOtherRadius(Other);

            if (FVector::Dist2D(MyPos, OtherLoc) < (UnitRadius + OtherRadius))
            {
                return ComputeSafestPoint(ClippingHits);
            }
        }
    }    
    return false;
}

bool UUnitMovementComponent::ComputeSafestPoint(TArray<FOverlapResult>& NearObjects)
{
    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(123, 3.0f, FColor::Orange, TEXT("ComputeSafestPoint"));
    }

    if (InternalPathPoints.Num() == 0)
        return false;

    FVector Desired = InternalPathPoints.Last();
    float Buffer = ExtraBuffer + (UnitRadius/2);
    FVector AccumulatedPush = FVector::ZeroVector;
    float RequiredMoveDist = 0.f;

    for (const FOverlapResult& Hit : NearObjects)
    {
        AActor* Other = Hit.GetActor();
        if (!Other || Other == CharacterOwner)
            continue;

        FVector OtherLoc = Other->GetActorLocation();

        // --- dynamic radii ---
        float OtherRadius = SetOtherRadius(Other);

        const float SafeDist = UnitRadius + OtherRadius + Buffer;

        FVector ToDesired = Desired - OtherLoc;
        float Dist2D = ToDesired.Size2D();

        // Only care if destination is invalid
        if (Dist2D < SafeDist)
        {
            FVector PushDir;

            if (Dist2D > KINDA_SMALL_NUMBER)
            {
                // Normal case
                PushDir = ToDesired.GetSafeNormal2D();
            }
            else
            {
                // EXACT CENTER CASE — fabricate direction
                FVector FromOtherToMe = CharacterOwner->GetActorLocation() - OtherLoc;
                PushDir = FromOtherToMe.GetSafeNormal2D();

                // Absolute fallback if somehow still zero
                if (PushDir.IsNearlyZero())
                {
                    PushDir = FVector::ForwardVector;
                }
            }

            AccumulatedPush += PushDir;

            float Needed = SafeDist - FMath::Max(Dist2D, 1.f);
            RequiredMoveDist = FMath::Max(RequiredMoveDist, Needed);
        }
    }

    // No overlaps → destination already safe
    if (AccumulatedPush.IsNearlyZero())
        return false;

    FVector FinalDir = AccumulatedPush.GetSafeNormal2D();
    FVector Adjusted = Desired + FinalDir * RequiredMoveDist;

    // Project to navmesh
    if (UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(GetWorld()))
    {
        FNavLocation Projected;
        if (NavSys->ProjectPointToNavigation(Adjusted, Projected))
        {
            InternalPathPoints.Last() = Projected.Location;
            return true;
        }
    }

    // Fallback if navmesh projection fails
    InternalPathPoints.Last() = Adjusted;
    return true;
}

float UUnitMovementComponent::SetOtherRadius(const AActor* Other)
{
    float OtherRadius = 30.f;
    if (UCapsuleComponent* OtherCap = Other->FindComponentByClass<UCapsuleComponent>())
    {
        OtherRadius = OtherCap->GetScaledCapsuleRadius();
    }
    else if (UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Other->GetRootComponent())) //if other root component has bounds (size)  
    {
        FVector E = RootPrim->Bounds.BoxExtent; //size of object
        OtherRadius = FMath::Max(E.X, E.Y); //choose larger value
    }
    return OtherRadius;
}

void UUnitMovementComponent::SmoothFaceDirection(const FVector& Direction, float DeltaTime)
{
    if (!CharacterOwner || Direction.IsNearlyZero())
        return;

    float ScaledRotationSpeed = RotationSpeed / RotationSizeScale;

    // --- movement speed scaling (polish) ---
    if (MoveComp)
    {
        const float SpeedRatio = MoveComp->Velocity.Size2D() / FMath::Max(MoveSpeed, 1.f);
        ScaledRotationSpeed *= FMath::Clamp(SpeedRatio, 0.5f, 1.f);
    }

    // --- setting rotation ---
    FRotator CurrentRot = CharacterOwner->GetActorRotation();
    FRotator TargetRot(0.f, Direction.Rotation().Yaw, 0.f);

    FRotator NewRot = FMath::RInterpConstantTo(CurrentRot, TargetRot, DeltaTime, ScaledRotationSpeed);
    CharacterOwner->SetActorRotation(NewRot);
}

void UUnitMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (bMoving)
        TickMoveAlongPath(DeltaTime);
}