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
    bool bGoalAdjusted = false;
    if (!CharacterOwner || !GetWorld()) return false;

    const float Range = FMath::Max(50.f, AvoidanceCheckDistance);

    const float InitialLockTime = 0.28f;
    const float ReplaceLockIfMuchCloserFactor = 0.65f;
    const float CriticalEdgeToBreakLock = 2.f;

    const float RecentBlockHoldTime = 0.18f;

    const FVector MyPos2D(MyPos.X, MyPos.Y, 0.f);

    const bool bHasPath = InternalPathPoints.Num() > 0;
    const bool bIsFinalPoint = bHasPath && (CurrentPathIndex >= InternalPathPoints.Num() - 1);

    FVector PointA3D = CurrentGoal;
    if (InternalPathPoints.IsValidIndex(CurrentPathIndex))
    {
        PointA3D = InternalPathPoints[CurrentPathIndex];
    }
    const FVector PointA2D(PointA3D.X, PointA3D.Y, 0.f);

    FVector DirA2D = PointA2D - MyPos2D;
    const float DistToA = DirA2D.Size();
    if (DistToA < KINDA_SMALL_NUMBER) return false;
    DirA2D /= DistToA;

    const float LenA = bIsFinalPoint ? DistToA : FMath::Min(Range, DistToA);
    const FVector StartA = MyPos2D;
    const FVector EndA = MyPos2D + DirA2D * LenA;

    const float Remaining = Range - FMath::Min(Range, LenA);

    float LenB = 0.f;
    FVector StartB = PointA2D;
    FVector EndB = PointA2D;

    if (Remaining > KINDA_SMALL_NUMBER && InternalPathPoints.IsValidIndex(CurrentPathIndex + 1))
    {
        const FVector PointB3D = InternalPathPoints[CurrentPathIndex + 1];
        const FVector PointB2D(PointB3D.X, PointB3D.Y, 0.f);

        const FVector AB = (PointB2D - PointA2D);
        const float DistAB = AB.Size();
        if (DistAB > KINDA_SMALL_NUMBER)
        {
            const FVector DirB2D = AB / DistAB;
            LenB = FMath::Min(Remaining, DistAB);
            StartB = PointA2D;
            EndB = PointA2D + DirB2D * LenB;
        }
    }

    TArray<FOverlapResult> Overlaps;
    FCollisionObjectQueryParams ObjQuery;
    ObjQuery.AddObjectTypesToQuery(ECC_Pawn);
    ObjQuery.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams Params(SCENE_QUERY_STAT(AvoidanceOverlap), true, CharacterOwner);

    const bool bAny = GetWorld()->OverlapMultiByObjectType(
        Overlaps,
        MyPos,
        FQuat::Identity,
        ObjQuery,
        FCollisionShape::MakeSphere(Range),
        Params
    );

    bool bDetectedAny = false;
    bool bInCorridor = false;
    bool bGoalBlocked = false;

    TArray<AActor*> BlockingActors;
    AActor* DominantBlocker = nullptr;

    float BestDomScore = -FLT_MAX;
    float BestDomDist2D = FLT_MAX;
    float BestDomCombinedRadius = 0.f;

    auto ClosestPointOnSegment2D = [&](const FVector& P, const FVector& S, const FVector& E, float& OutAlong, FVector& OutClosest)
    {
        const FVector SE = (E - S);
        const float SegLen = SE.Size();
        if (SegLen < KINDA_SMALL_NUMBER)
        {
            OutAlong = 0.f;
            OutClosest = S;
            return;
        }

        const FVector Dir = SE / SegLen;
        float t = FVector::DotProduct((P - S), Dir);
        t = FMath::Clamp(t, 0.f, SegLen);
        OutAlong = t;
        OutClosest = S + Dir * t;
    };

    auto TestSegment = [&](AActor* Other, const FVector& SegStart, const FVector& SegEnd, float SegLen, float CombinedRadius,
        bool& bOutInFront, float& OutAlong, float& OutPerp) -> bool
    {
        bOutInFront = false;
        OutAlong = 0.f;
        OutPerp = FLT_MAX;

        if (!Other || SegLen < KINDA_SMALL_NUMBER) return false;

        const FVector OtherLoc = Other->GetActorLocation();
        const FVector Other2D(OtherLoc.X, OtherLoc.Y, 0.f);

        FVector Closest = SegStart;
        ClosestPointOnSegment2D(Other2D, SegStart, SegEnd, OutAlong, Closest);

        const FVector SegDir = (SegEnd - SegStart).GetSafeNormal();
        const float RawAlong = FVector::DotProduct((Other2D - SegStart), SegDir);
        if (RawAlong > 0.f && RawAlong <= SegLen + 1.f) bOutInFront = true;

        OutPerp = FVector::Dist(Other2D, Closest);
        return (bOutInFront && OutPerp <= CombinedRadius);
    };

    auto ScoreDominant = [&](float Along, float Perp, float CombinedRadius) -> float
    {
        const float AlongN = 1.f - FMath::Clamp(Along / FMath::Max(Range, 1.f), 0.f, 1.f);
        const float PerpN = 1.f - FMath::Clamp(Perp / FMath::Max(CombinedRadius, 1.f), 0.f, 1.f);
        return AlongN * 2.0f + PerpN * 1.0f;
    };

    if (bAny && Overlaps.Num() > 0)
    {
        for (const FOverlapResult& R : Overlaps)
        {
            AActor* Other = R.GetActor();
            if (!Other || Other == CharacterOwner) continue;
            if (MoveTargetActor.IsValid() && Other == MoveTargetActor.Get()) continue;

            bDetectedAny = true;

            const float OtherRadius = SetOtherRadius(Other);
            const float CombinedRadius = UnitRadius + OtherRadius + ExtraBuffer;

            const FVector OtherLoc = Other->GetActorLocation();
            const FVector Other2D(OtherLoc.X, OtherLoc.Y, 0.f);

            //checking if soft collision should run this tick
            const float GateRange = CombinedRadius + SoftGateBuffer;
            const float DistSq2D = FVector::DistSquared2D(MyPos2D, Other2D);
            if (DistSq2D <= FMath::Square(GateRange))
            {
                bUnitInRangeForSoft = true;
            }

            // Goal-block check (separate from corridor)
            if (bIsFinalPoint)
            {
                const float GoalDist = FVector::Dist(Other2D, PointA2D);
                if (GoalDist <= CombinedRadius)
                {
                    bGoalBlocked = true;
                    BlockingActors.AddUnique(Other);
                }
            }

            bool bInFrontA = false, bInFrontB = false;
            float AlongA = 0.f, PerpA = FLT_MAX;
            float AlongB = 0.f, PerpB = FLT_MAX;

            const bool bBlockA = TestSegment(Other, StartA, EndA, LenA, CombinedRadius, bInFrontA, AlongA, PerpA);
            const bool bBlockB = (LenB > 0.f) ? TestSegment(Other, StartB, EndB, LenB, CombinedRadius, bInFrontB, AlongB, PerpB) : false;

            if (bBlockA || bBlockB)
            {
                bInCorridor = true;
                BlockingActors.AddUnique(Other);

                const float UseAlong = bBlockA ? AlongA : AlongB;
                const float UsePerp = bBlockA ? PerpA : PerpB;

                const float Score = ScoreDominant(UseAlong, UsePerp, CombinedRadius);
                const float Dist2D = FVector::Dist2D(MyPos, OtherLoc);

                if (Score > BestDomScore || (FMath::IsNearlyEqual(Score, BestDomScore, 0.01f) && Dist2D < BestDomDist2D))
                {
                    BestDomScore = Score;
                    BestDomDist2D = Dist2D;
                    BestDomCombinedRadius = CombinedRadius;
                    DominantBlocker = Other;
                }
            }
        }
    }

    // GoalAdjustment should be driven by goal-block, not corridor-block
    if (bGoalBlocked && bIsFinalPoint)
    {
        bGoalAdjusted = GoalAdjustment();
        if (bGoalAdjusted)
        {
            return true;
        }
    }

    // Recent-block hold: refresh if corridor-block OR goal-block
    if (bInCorridor || bGoalBlocked)
    {
        Avoid_RecentBlockRemaining = RecentBlockHoldTime;
    }
    else
    {
        Avoid_RecentBlockRemaining = FMath::Max(0.f, Avoid_RecentBlockRemaining - DeltaTime);
    }
    const bool bRecentlyBlocked = (Avoid_RecentBlockRemaining > 0.f);

    const bool bInitialAvoid = (bInCorridor || bGoalBlocked || bRecentlyBlocked);

    // Safety lock
    Avoid_InitialLockRemaining = FMath::Max(0.f, Avoid_InitialLockRemaining - DeltaTime);

    const bool bHasLock = (Avoid_InitialLockRemaining > 0.f) && Avoid_LockedDominant.IsValid();
    AActor* LockedActor = bHasLock ? Avoid_LockedDominant.Get() : nullptr;

    if ((SteeringAuthority == ESteeringAuthority::SoftCollision) && LockedActor && DominantBlocker && (DominantBlocker != LockedActor))
    {
        SteeringAuthority = ESteeringAuthority::PathFollowing;
    }

    auto AcquireLock = [&]()
    {
        Avoid_LockedDominant = DominantBlocker;
        Avoid_InitialLockRemaining = InitialLockTime;
        LockedActor = DominantBlocker;
    };

    if (bInitialAvoid)
    {
        if (!bHasLock)
        {
            AcquireLock();
        }
        else
        {
            bool bBreakLock = false;

            if (!LockedActor) bBreakLock = true;

            if (!bBreakLock && DominantBlocker)
            {
                const float DominantDist2D = FVector::Dist2D(MyPos, DominantBlocker->GetActorLocation());
                const float DominantEdge2D = DominantDist2D - BestDomCombinedRadius;
                if (DominantEdge2D <= CriticalEdgeToBreakLock) bBreakLock = true;
            }

            if (!bBreakLock && DominantBlocker && LockedActor && DominantBlocker != LockedActor)
            {
                const float LockedDist = FVector::Dist2D(MyPos, LockedActor->GetActorLocation());
                const float NewDist = FVector::Dist2D(MyPos, DominantBlocker->GetActorLocation());
                if (NewDist <= LockedDist * ReplaceLockIfMuchCloserFactor) bBreakLock = true;
            }

            if (bBreakLock)
            {
                AcquireLock();
            }
        }
    }
    else
    {
        Avoid_LockedDominant = nullptr;
        Avoid_InitialLockRemaining = 0.f;
        LockedActor = nullptr;
    }

    if (SteeringAuthority != ESteeringAuthority::SoftCollision)
    {
        AActor* EffectiveDominant = LockedActor ? LockedActor : DominantBlocker;

        if (bInitialAvoid && EffectiveDominant)
        {
            //InitialAvoidance(DesiredDir, MyPos, EffectiveDominant);
        }
    }

    //------------------Debug------------------
    if (bInitialAvoid) ForwardColor = FColor(255, 165, 0);
    else if (bDetectedAny) ForwardColor = FColor::Blue;
    else ForwardColor = FColor::Green;
    //------------------Debug------------------

    return bGoalAdjusted;
}

// New InitialAvoidance Version 4
void UUnitMovementComponent::InitialAvoidance(FVector& DesiredDir, const FVector& MyPos, AActor* DominantBlocker)
{
    if (!CharacterOwner || !DominantBlocker) return;

    const float DeltaTime = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.f;
    if (DeltaTime <= 0.f) return;

    FVector PathDir2D(DesiredDir.X, DesiredDir.Y, 0.f);
    if (PathDir2D.IsNearlyZero()) return;
    PathDir2D.Normalize();

    const FVector MyPos2D(MyPos.X, MyPos.Y, 0.f);
    const FVector OtherLoc = DominantBlocker->GetActorLocation();
    const FVector OtherPos2D(OtherLoc.X, OtherLoc.Y, 0.f);

    FVector ToOther2D = OtherPos2D - MyPos2D;
    const float Dist = ToOther2D.Size();
    if (Dist < KINDA_SMALL_NUMBER) return;

    const FVector ToOtherN = ToOther2D / Dist;

    const float OtherRadius = SetOtherRadius(DominantBlocker);
    const float CombinedRadius = UnitRadius + OtherRadius + ExtraBuffer;

    // Window timing (same feel as v2/v3)
    const float ReactionTime = 0.35f;
    const float StartBuffer = 20.f;
    const float MaxExtraStart = 180.f;

    const FVector MyVel2D(CharacterOwner->GetVelocity().X, CharacterOwner->GetVelocity().Y, 0.f);
    const FVector OtherVel2D(DominantBlocker->GetVelocity().X, DominantBlocker->GetVelocity().Y, 0.f);
    const FVector RelVel2D = MyVel2D - OtherVel2D;

    const float ApproachSpeed = FMath::Max(0.f, FVector::DotProduct(RelVel2D, ToOtherN));
    const float ExtraStart = FMath::Clamp(ApproachSpeed * ReactionTime, 0.f, MaxExtraStart);

    const float TriggerDist = CombinedRadius + StartBuffer + ExtraStart;

    // Hysteresis + smooth release (prevents snap/thrash)
    const float ReleaseDistPad = 28.f;
    const float ReleaseAheadOn = 0.06f;   // must be at least this “ahead” to keep engaging
    const float ReleaseAheadOff = -0.08f;  // must be behind by this to force release
    const float ReleaseBlendTime = 0.10f;  // seconds to blend back to path after release

    // If outside band entirely: hard clear
    if (Dist > TriggerDist + ReleaseDistPad)
    {
        IA_Target = nullptr;
        IA_SideSign = 0;
        IA_MinDist = BIG_NUMBER;
        IA_BaseDir2D = FVector::ZeroVector;
        IA_ReleaseAlpha = 1.f;
        IA_bReleasing = false;
        return;
    }

    // Start / maintain engagement state
    const bool bNewTarget = (!IA_Target.IsValid() || IA_Target.Get() != DominantBlocker);

    if (bNewTarget)
    {
        IA_Target = DominantBlocker;
        IA_SideSign = 0;
        IA_MinDist = Dist;
        IA_BaseDir2D = PathDir2D;   // lock base direction for the engagement
        IA_bReleasing = false;
        IA_ReleaseAlpha = 1.f;
    }

    // Decide whether we should be releasing (shoulder-to-shoulder / passed)
    const FVector BaseDir2D = IA_BaseDir2D.IsNearlyZero() ? PathDir2D : IA_BaseDir2D;
    const float Ahead = FVector::DotProduct(ToOtherN, BaseDir2D);

    // If already releasing, keep blending out
    if (IA_bReleasing)
    {
        IA_ReleaseAlpha = FMath::Min(1.f, IA_ReleaseAlpha + (DeltaTime / FMath::Max(ReleaseBlendTime, 0.01f)));
        const FVector BlendDir = FMath::Lerp(IA_LastAvoidDir2D, PathDir2D, IA_ReleaseAlpha).GetSafeNormal();
        DesiredDir.X = BlendDir.X;
        DesiredDir.Y = BlendDir.Y;
        DesiredDir.Z = 0.f;

        if (IA_ReleaseAlpha >= 1.f)
        {
            IA_Target = nullptr;
            IA_SideSign = 0;
            IA_MinDist = BIG_NUMBER;
            IA_BaseDir2D = FVector::ZeroVector;
            IA_bReleasing = false;
        }
        return;
    }

    // Force release only when clearly behind (hysteresis)
    if (Ahead <= ReleaseAheadOff)
    {
        IA_bReleasing = true;
        IA_ReleaseAlpha = 0.f;
        IA_LastAvoidDir2D = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
        return;
    }

    // If not sufficiently ahead, do NOT recompute new avoidance; keep current direction (prevents flicker)
    if (Ahead < ReleaseAheadOn)
    {
        return;
    }

    // Monotonic progress driver
    IA_MinDist = FMath::Min(IA_MinDist, Dist);

    const float Den = FMath::Max(TriggerDist - CombinedRadius, 1.f);
    float t = (TriggerDist - IA_MinDist) / Den;
    t = FMath::Clamp(t, 0.f, 1.f);

    const float Curve = t * t;

    // Lock side once
    if (IA_SideSign == 0)
    {
        const FVector TangentL = FVector::CrossProduct(FVector::UpVector, ToOtherN).GetSafeNormal();
        const FVector TangentR = -TangentL;
        IA_SideSign = (FVector::DotProduct(TangentL, BaseDir2D) >= FVector::DotProduct(TangentR, BaseDir2D)) ? +1 : -1;
    }

    // Slightly wider than v2, but not huge
    const float SizeRef = 45.f;
    const float SizeScale = FMath::Clamp(OtherRadius / FMath::Max(SizeRef, 1.f), 0.90f, 1.20f);

    const float MaxArcDegBase = 38.f;
    const float ExtraArcDegBase = 6.f;

    const float MaxArcDeg = MaxArcDegBase * SizeScale;
    const float ExtraArcDeg = ExtraArcDegBase * SizeScale;

    const float ExtraRamp = FMath::SmoothStep(0.f, 1.f, FMath::Clamp(t / 0.55f, 0.f, 1.f));
    const float ArcDeg = (MaxArcDeg * Curve) + (ExtraArcDeg * ExtraRamp);

    FVector NewDir2D = BaseDir2D.RotateAngleAxis(ArcDeg * (float)IA_SideSign, FVector::UpVector);
    NewDir2D.Z = 0.f;
    NewDir2D.Normalize();

    IA_LastAvoidDir2D = NewDir2D;

    SteeringAuthority = ESteeringAuthority::InitialAvoidance;

    DesiredDir.X = NewDir2D.X;
    DesiredDir.Y = NewDir2D.Y;
    DesiredDir.Z = 0.f;
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