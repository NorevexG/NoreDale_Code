// UnitMovementComponent.cpp
//To Do befor calling this finished:
// - remove debug code from avoidance check
// - rework avoidance check and apply avoidance

#include "UnitMovementComponent.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "NavigationSystem.h"
#include "NavigationPath.h"
#include "DrawDebugHelpers.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
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

    if (SteeringAuthority != ESteeringAuthority::SoftCollision)
    {
        SteeringAuthority = ESteeringAuthority::PathFollowing;
    }

    const bool bGoalAdjusted = AvoidanceCheck(DesiredDir, Pos, DeltaTime);
    if (bGoalAdjusted)
        return false;

    FVector SoftMoveDir = FVector::ZeroVector;
    const bool bSoft = SoftCollision(DesiredDir, Pos, DeltaTime, SoftMoveDir);
    FVector FinalDir = (bSoft && !SoftMoveDir.IsNearlyZero()) ? SoftMoveDir : DesiredDir;
    if (FinalDir.IsNearlyZero())
        FinalDir = DesiredDir;

    if (bSoft)
    {
        SteeringAuthority = ESteeringAuthority::SoftCollision;
    }
    else
    {
        SteeringAuthority = ESteeringAuthority::PathFollowing;
        SoftCollisionActor = nullptr;
    }

    UpdateEmergencyDetection(DesiredDir, FinalDir, Pos, DeltaTime, bSoft);
    if (bEmergencyDetected)
    {
        ForwardColor = FColor::Red;
    }
    DebugTools(FinalDir, Pos);

    if (MoveComp)
    {
        if (MoveComp->MovementMode == MOVE_None)
            MoveComp->SetMovementMode(MOVE_Walking);

        MoveComp->MaxWalkSpeed = MoveSpeed;
    }

    CharacterOwner->AddMovementInput(FinalDir, 1.f);
    SmoothFaceDirection(FinalDir, DeltaTime);

    return false;
}

//DebugTools
void UUnitMovementComponent::DebugTools(const FVector& DesiredDir, const FVector& MyPos)
{
    // Clipping debug (visual only)
    if (CapComp)
    {
        const FVector CapCenter = CapComp->GetComponentLocation();
        const float CapRadius = CapComp->GetScaledCapsuleRadius();
        const float CapHalfHeight = CapComp->GetScaledCapsuleHalfHeight();

        TArray<FOverlapResult> ClipHits;
        FCollisionQueryParams ClipParams(SCENE_QUERY_STAT(ClippingOverlap), true, CharacterOwner);

        const bool bClipAny = GetWorld()->OverlapMultiByObjectType(
            ClipHits,
            CapCenter,
            FQuat::Identity,
            FCollisionObjectQueryParams(ECC_Pawn),
            FCollisionShape::MakeCapsule(CapRadius, CapHalfHeight),
            ClipParams
        );

        if (bClipAny)
        {
            for (const FOverlapResult& CH : ClipHits)
            {
                AActor* Other = CH.GetActor();
                if (!Other || Other == CharacterOwner) continue;

                DrawDebugCapsule(GetWorld(), CapCenter, CapHalfHeight, CapRadius, FQuat::Identity, FColor::Red, false, 0.05f, 0, 2.f);
                DrawDebugLine(GetWorld(), MyPos, Other->GetActorLocation(), FColor::Red, false, 0.05f, 0, 3.f);
            }
        }
    }

    if (bDebugAvoidance)
    {
        float Z = CharacterOwner->GetActorLocation().Z;
        if (CapComp)
        {
            Z += CapComp->GetScaledCapsuleHalfHeight() * 0.5f;
        }

        FVector Start = MyPos; Start.Z = Z;

        FVector Dir = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
        if (Dir.IsNearlyZero())
        {
            return;
        }

        const float Range = FMath::Max(50.f, AvoidanceCheckDistance);
        const FVector End = Start + Dir * Range;
        DrawDebugLine(GetWorld(), Start, End, ForwardColor, false, 0.05f, 0, 4.f);
    }
}

bool UUnitMovementComponent::AvoidanceCheck(FVector& DesiredDir, const FVector& MyPos, float DeltaTime)
{
    bUnitInRangeForSoft = false;

    if (!CharacterOwner || !GetWorld())
    {
        ForwardColor = FColor::Green;
        return false;
    }

    bool bGoalAdjusted = false;

    const float Range = FMath::Max(50.f, AvoidanceCheckDistance);

    const FVector MyPos2D(MyPos.X, MyPos.Y, 0.f);
    const bool bHasPath = InternalPathPoints.Num() > 0;
    const bool bIsFinalPoint = bHasPath && (CurrentPathIndex >= InternalPathPoints.Num() - 1);

    FVector Goal3D = CurrentGoal;
    if (InternalPathPoints.IsValidIndex(CurrentPathIndex))
    {
        Goal3D = InternalPathPoints[CurrentPathIndex];
    }

    const FVector Goal2D(Goal3D.X, Goal3D.Y, 0.f);
    const FVector ToGoalVec = (Goal2D - MyPos2D);
    const float DistToGoal = ToGoalVec.Size();

    if (DistToGoal < KINDA_SMALL_NUMBER)
    {
        ForwardColor = FColor::Green;
        return false;
    }

    const FVector DirToGoal2D = ToGoalVec / DistToGoal;

    TArray<FOverlapResult> Overlaps;
    FCollisionObjectQueryParams ObjQuery;
    ObjQuery.AddObjectTypesToQuery(ECC_Pawn);
    ObjQuery.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams Params(SCENE_QUERY_STAT(AvoidanceOverlap), true, CharacterOwner);

    GetWorld()->OverlapMultiByObjectType(
        Overlaps,
        MyPos,
        FQuat::Identity,
        ObjQuery,
        FCollisionShape::MakeSphere(Range),
        Params
    );

    bool bDetectedAny = false;
    bool bGoalBlocked = false;

    for (const FOverlapResult& Res : Overlaps)
    {
        AActor* Other = Res.GetActor();
        if (!Other || Other == CharacterOwner)
        {
            continue;
        }

        bDetectedAny = true;

        const FVector OtherLoc = Other->GetActorLocation();
        const FVector Other2D(OtherLoc.X, OtherLoc.Y, 0.f);

        const float OtherR = SetOtherRadius(Other);
        const float CombinedR = UnitRadius + OtherR + ExtraBuffer;

        const float Dist2D = FVector::Dist2D(MyPos, OtherLoc);

        const float SoftEnter = UnitRadius + OtherR + SoftGateBuffer;
        if (Dist2D <= SoftEnter)
        {
            bUnitInRangeForSoft = true;
        }
        if (bIsFinalPoint)
        {
            const FVector ToOther = (Other2D - MyPos2D);
            const float Along = FVector::DotProduct(ToOther, DirToGoal2D);
            const float ClampedAlong = FMath::Clamp(Along, 0.f, DistToGoal);
            const FVector Closest = MyPos2D + DirToGoal2D * ClampedAlong;

            const float Perp = (Other2D - Closest).Size();
            if (Perp <= CombinedR && Along >= -CombinedR && Along <= (DistToGoal + CombinedR))
            {
                bGoalBlocked = true;
            }
        }
    }
    if (bGoalBlocked && bIsFinalPoint)
    {
        bGoalAdjusted = GoalAdjustment();
        if (bGoalAdjusted)
        {
            ForwardColor = bDetectedAny ? FColor::Blue : FColor::Green;
            return true;
        }
    }

    // -=-=-=-=-=-=- EmergancyAvoidance -=-=-=-=-=-=-
    const bool bEmergencyNeeded = false;
    // -=-=-=-=-=-=- EmergancyAvoidance -=-=-=-=-=-=-

    // -=-=-=-=-=-=-=-=-=- Debug -=-=-=-=-=-=-=-=-=-
    if (bDetectedAny)
    {
        ForwardColor = FColor::Blue;
    }
    else
    {
        ForwardColor = FColor::Green;
    }
    // -=-=-=-=-=-=-=-=-=- Debug -=-=-=-=-=-=-=-=-=-

    return bGoalAdjusted;
}

//New SoftCollision V6
bool UUnitMovementComponent::SoftCollision(const FVector& DesiredDir, const FVector& MyPos, float DeltaTime, FVector& OutMoveDir2D)
{
    OutMoveDir2D = FVector::ZeroVector;

    if (!bEnableSoftCollision || !CharacterOwner || !CapComp)
        return false;

    if (!bUnitInRangeForSoft && SteeringAuthority != ESteeringAuthority::SoftCollision)
        return false;

    FVector Desired2D(DesiredDir.X, DesiredDir.Y, 0.f);
    if (Desired2D.IsNearlyZero())
        return false;

    Desired2D.Normalize();

    // --- Sweep forward to find blocker (same as V1-style)
    const FVector CapCenter = CapComp->GetComponentLocation();
    const float CapRadius = CapComp->GetScaledCapsuleRadius();
    const float CapHalf = CapComp->GetScaledCapsuleHalfHeight();

    const float StepDist = FMath::Max(2.f, MoveSpeed * DeltaTime);
    const FVector Start = CapCenter;
    const FVector End = CapCenter + Desired2D * StepDist;

    FCollisionQueryParams Params(SCENE_QUERY_STAT(SoftCollision), true, CharacterOwner);
    Params.AddIgnoredActor(CharacterOwner);

    FCollisionObjectQueryParams ObjParams;
    ObjParams.AddObjectTypesToQuery(ECC_Pawn);
    ObjParams.AddObjectTypesToQuery(ECC_PhysicsBody);

    FHitResult Hit;
    const bool bHit = GetWorld()->SweepSingleByObjectType(
        Hit,
        Start,
        End,
        CapComp->GetComponentQuat(),
        ObjParams,
        FCollisionShape::MakeCapsule(CapRadius, CapHalf),
        Params);

    AActor* Other = bHit ? Hit.GetActor() : nullptr;

    if (!Other) //if we didnt hit something
    {
        if (SteeringAuthority != ESteeringAuthority::SoftCollision)
            return false;

        // overlap-sticky at CapCenter (slightly inflated)
        TArray<FOverlapResult> Overlaps;
        const float Inflate = 6.f;
        const float R = CapRadius + Inflate;

        FCollisionQueryParams QParams(SCENE_QUERY_STAT(SoftCollisionSticky), true, CharacterOwner);
        QParams.AddIgnoredActor(CharacterOwner);

        const bool bAny = GetWorld()->OverlapMultiByObjectType(
            Overlaps, CapCenter, FQuat::Identity,
            ObjParams,
            FCollisionShape::MakeCapsule(R, CapHalf),
            QParams
        );
        if (!bAny) return false;

        for (const FOverlapResult& O : Overlaps)
        {
            AActor* A = O.GetActor();
            if (!A) continue;
            if (MoveTargetActor.IsValid() && A == MoveTargetActor.Get()) continue;
            Other = A;
            break;
        }
        if (!Other) return false;
    }

    if (!Other || (MoveTargetActor.IsValid() && Other == MoveTargetActor.Get()))
        return false;

    FVector ToOther2D = FVector(
        Other->GetActorLocation().X - CapCenter.X,
        Other->GetActorLocation().Y - CapCenter.Y,
        0.f);

    if (ToOther2D.IsNearlyZero())
        return false;

    ToOther2D.Normalize();

    // If we are NOT steering into the unit anymore → RELEASE
    if (FVector::DotProduct(Desired2D, ToOther2D) <= 0.f)
        return false;

    // --- Stable slide direction (no Hit.Normal)
    FVector Slide2D = FVector(-ToOther2D.Y, ToOther2D.X, 0.f) * (float)AvoidanceSide;
    Slide2D.Normalize();

    SoftCollisionActor = Other;

    OutMoveDir2D = Slide2D;

    return true;
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
    if (InternalPathPoints.Num() == 0)
        return false;

    FVector Desired = InternalPathPoints.Last();
    float Buffer = ExtraBuffer + (UnitRadius / 2);
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


// ---------- Emergency detection (PURE DETECTION ONLY; no steering/execution) ----------
static float NormalizeAngleRad(float AngleRad)
{
    // Normalize to (-pi, pi]
    AngleRad = FMath::Fmod(AngleRad, 2.f * PI);
    if (AngleRad <= -PI) AngleRad += 2.f * PI;
    if (AngleRad > PI) AngleRad -= 2.f * PI;
    return AngleRad;
}

void UUnitMovementComponent::ResetEmergencyDetection()
{
    bEmergencyDetected = false;

    Emergency_TrackedBlocker = nullptr;
    Emergency_bHasPrevRelAngle = false;
    Emergency_PrevRelAngleRad = 0.f;
    Emergency_OrbitSignedRad = 0.f;
    Emergency_AngleStableTime = 0.f;

    Emergency_bHasPrevPos2D = false;
    Emergency_PrevPos2D = FVector2D::ZeroVector;

    Emergency_PrevSideSign = 0;
    Emergency_FlipToggleCount = 0;
    Emergency_FlipWindowTime = 0.f;

    Emergency_ProgressSamples.Reset();
    Emergency_ProgressWindowTime = 0.f;
    Emergency_ProgressWindowSum = 0.f;
}

void UUnitMovementComponent::UpdateEmergencyDetection(
    const FVector& DesiredDir,
    const FVector& FinalDir,
    const FVector& MyPos,
    float DeltaTime,
    bool /*bSoftCollisionActive*/
)
{
    if (!bEnableEmergencyDetection || DeltaTime <= 0.f)
    {
        bEmergencyDetected = false;
        return;
    }

    // We need a blocker to define obstacle-relative angles. If none, reset state.
    AActor* Blocker = SoftCollisionActor.Get();
    if (!Blocker)
    {
        ResetEmergencyDetection();
        return;
    }

    // If blocker changed, reset tracking (we don't want mixed histories).
    if (Emergency_TrackedBlocker.Get() != Blocker)
    {
        ResetEmergencyDetection();
        Emergency_TrackedBlocker = Blocker;
    }

    const FVector BlockerPos = Blocker->GetActorLocation();
    const FVector2D MyPos2D(MyPos.X, MyPos.Y);
    const FVector2D BlockerPos2D(BlockerPos.X, BlockerPos.Y);
    const FVector2D ToBlocker2D = (BlockerPos2D - MyPos2D);
    if (ToBlocker2D.IsNearlyZero())
    {
        // Degenerate (same position). Treat as no detection this frame but keep blocker.
        bEmergencyDetected = false;
        return;
    }

    // --- Relative angle tracking (Atan2) ---
    const float RelAngleRad = FMath::Atan2(MyPos2D.Y - BlockerPos2D.Y, MyPos2D.X - BlockerPos2D.X);

    if (!Emergency_bHasPrevRelAngle)
    {
        Emergency_bHasPrevRelAngle = true;
        Emergency_PrevRelAngleRad = RelAngleRad;
    }

    const float DeltaAngle = NormalizeAngleRad(RelAngleRad - Emergency_PrevRelAngleRad);
    Emergency_PrevRelAngleRad = RelAngleRad;

    // Signed orbit accumulation: back-and-forth cancels out.
    Emergency_OrbitSignedRad += DeltaAngle;

    // Deadlock: angle barely changes for a sustained period.
    if (FMath::Abs(DeltaAngle) <= Emergency_SmallAngleEpsRad)
    {
        Emergency_AngleStableTime += DeltaTime;
    }
    else
    {
        Emergency_AngleStableTime = 0.f;
        // When we clearly move around the blocker, decay orbit accumulation a bit to avoid runaway over long paths.
        Emergency_OrbitSignedRad *= 0.98f;
    }

    const bool bDeadlocked = (Emergency_AngleStableTime >= Emergency_DeadlockStableSec);
    const bool bSpinning = (FMath::Abs(Emergency_OrbitSignedRad) >= Emergency_SpinOrbitRad);

    // --- Flip-flop detection: sign toggles in a short window ---
    FVector Desired2D(DesiredDir.X, DesiredDir.Y, 0.f);
    if (Desired2D.IsNearlyZero())
    {
        Desired2D = FVector(FinalDir.X, FinalDir.Y, 0.f);
    }
    Desired2D = Desired2D.GetSafeNormal();

    const FVector ToBlocker3D(ToBlocker2D.X, ToBlocker2D.Y, 0.f);
    const float CrossZ = (Desired2D.X * ToBlocker3D.Y) - (Desired2D.Y * ToBlocker3D.X);
    const int32 SideSign = (CrossZ >= 0.f) ? 1 : -1;

    if (Emergency_PrevSideSign == 0)
    {
        Emergency_PrevSideSign = SideSign;
    }
    else if (SideSign != Emergency_PrevSideSign)
    {
        Emergency_FlipToggleCount++;
        Emergency_PrevSideSign = SideSign;
    }

    Emergency_FlipWindowTime += DeltaTime;
    if (Emergency_FlipWindowTime >= Emergency_FlipWindowSec)
    {
        Emergency_FlipWindowTime = 0.f;
        Emergency_FlipToggleCount = 0;
        Emergency_PrevSideSign = SideSign;
    }

    const bool bFlipFlop = (Emergency_FlipToggleCount >= Emergency_FlipToggleThreshold);

    // --- Progress safety net (rolling time window) ---
    const FVector2D DeltaPos2D = Emergency_bHasPrevPos2D ? (MyPos2D - Emergency_PrevPos2D) : FVector2D::ZeroVector;
    Emergency_PrevPos2D = MyPos2D;
    Emergency_bHasPrevPos2D = true;

    const float ForwardThisFrame = FVector2D::DotProduct(DeltaPos2D, FVector2D(Desired2D.X, Desired2D.Y));
    Emergency_ProgressSamples.Add({ DeltaTime, ForwardThisFrame });
    Emergency_ProgressWindowTime += DeltaTime;
    Emergency_ProgressWindowSum += ForwardThisFrame;

    while (Emergency_ProgressSamples.Num() > 0 && Emergency_ProgressWindowTime > Emergency_ProgressWindowSec)
    {
        const FEmergencyProgressSample& Old = Emergency_ProgressSamples[0];
        Emergency_ProgressWindowTime -= Old.Dt;
        Emergency_ProgressWindowSum -= Old.Forward;
        Emergency_ProgressSamples.RemoveAt(0, 1, false);
    }

    const bool bProgressWindowReady = (Emergency_ProgressWindowTime >= (Emergency_ProgressWindowSec * 0.8f));
    const bool bProgressFail = bProgressWindowReady && (Emergency_ProgressWindowSum < Emergency_MinProgressUnits);

    // Deadlock bypasses progress gate (per Z3 fix).
    const bool bEmergencyNeeded = bDeadlocked || ((bFlipFlop || bSpinning) && bProgressFail);

    bEmergencyDetected = bEmergencyNeeded;
}
// ---------- Emergency detection ----------

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