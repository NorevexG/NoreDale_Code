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

    bool bGoalAdjusted = AvoidanceCheck(DesiredDir, Pos, DeltaTime);
    if (bGoalAdjusted)
        return false;

    SoftCollision(DesiredDir, Pos, DeltaTime);

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

// New AvoidanceCheck Version 6.1
bool UUnitMovementComponent::AvoidanceCheck(FVector& DesiredDir, const FVector& MyPos, float DeltaTime)
{
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

            // Goal-block check (separate from corridor)
            if (bIsFinalPoint)
            {
                const float GoalDist = FVector::Dist(Other2D, PointA2D);
                if (GoalDist <= CombinedRadius)
                {
                    bGoalBlocked = true;
                    BlockingActors.AddUnique(Other);
                    // DO NOT continue; still allow corridor classification too if you want
                    // (but it’s fine either way). We’ll still corridor-test below.
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

    // Clipping debug stays (not used for state, just visual)
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

    AActor* EffectiveDominant = LockedActor ? LockedActor : DominantBlocker;

    if (bInitialAvoid && EffectiveDominant)
    {
        // InitialAvoidance(DesiredDir, MyPos, EffectiveDominant);
    }

    // Debug line: green / blue / orange
    if (bDebugAvoidance)
    {
        FColor ForwardColor = FColor::Green;
        if (bInitialAvoid) ForwardColor = FColor(255, 165, 0);
        else if (bDetectedAny) ForwardColor = FColor::Blue;

        float Z = CharacterOwner->GetActorLocation().Z;
        if (CapComp)
        {
            Z += CapComp->GetScaledCapsuleHalfHeight() * 0.5f;
        }

        FVector Start = MyPos; Start.Z = Z;

        FVector Dir = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
        if (Dir.IsNearlyZero())
        {
            Dir = DirA2D.GetSafeNormal();
        }

        const FVector End = Start + Dir * Range;
        DrawDebugLine(GetWorld(), Start, End, ForwardColor, false, 0.05f, 0, 4.f);
    }

    return bGoalAdjusted;
}

//SoftCollision - V10
void UUnitMovementComponent::SoftCollision(FVector& InOutDesiredDir, const FVector& MyPos, float DeltaTime)
{
    if (!bEnableSoftCollision) return;
    if (!CharacterOwner || !CapComp || !GetWorld()) return;

    // Forward intent (2D)
    FVector Forward2D(InOutDesiredDir.X, InOutDesiredDir.Y, 0.f);
    if (Forward2D.IsNearlyZero()) return;
    Forward2D = Forward2D.GetSafeNormal();

    const FVector CapCenter = CapComp->GetComponentLocation();
    const float   CapRadius = CapComp->GetScaledCapsuleRadius();
    const float   CapHalf = CapComp->GetScaledCapsuleHalfHeight();

    // Early warning buffer (keep aggressive-ish but not insane)
    const float BasePre = FMath::Clamp(UnitRadius * 0.20f, 6.f, 16.f);
    const float PreBuffer = BasePre;
    const float QueryRadius = CapRadius + PreBuffer;

    // Overlap query
    TArray<FOverlapResult> Hits;
    FCollisionQueryParams Params(SCENE_QUERY_STAT(SoftCollisionOverlap), true, CharacterOwner);
    FCollisionObjectQueryParams ObjQuery;
    ObjQuery.AddObjectTypesToQuery(ECC_Pawn);

    const bool bAny = GetWorld()->OverlapMultiByObjectType(
        Hits,
        CapCenter,
        FQuat::Identity,
        ObjQuery,
        FCollisionShape::MakeCapsule(QueryRadius, CapHalf),
        Params
    );

    if (!bAny || Hits.Num() == 0) return;

    // Spacing buffer (log10)
    const float SafeRadius = FMath::Max(UnitRadius, 1.f);
    const float Buffer = ExtraBuffer + (5.f * FMath::LogX(10.f, SafeRadius));

    FVector SepSum2D = FVector::ZeroVector;
    float TotalW = 0.f;

    bool bAnyTrueOverlap = false;
    float MaxOverlapT = 0.f;
    float MaxPreT = 0.f;

    FVector StrongestAway2D = FVector::ZeroVector;
    float StrongestPen = 0.f;

    for (const FOverlapResult& H : Hits)
    {
        AActor* Other = H.GetActor();
        if (!Other || Other == CharacterOwner) continue;
        if (MoveTargetActor.IsValid() && Other == MoveTargetActor.Get()) continue;

        const float OtherRadius = SetOtherRadius(Other);
        const float DesiredSep = UnitRadius + OtherRadius + Buffer;
        const float ReactDist = DesiredSep + PreBuffer;

        const FVector OtherPos = Other->GetActorLocation();
        FVector Delta2D(CapCenter.X - OtherPos.X, CapCenter.Y - OtherPos.Y, 0.f);

        float Dist2D = Delta2D.Size();
        if (Dist2D < KINDA_SMALL_NUMBER)
        {
            Delta2D = FVector(0.f, (float)AvoidanceSide, 0.f);
            Dist2D = 1.f;
        }

        if (Dist2D >= ReactDist) continue;

        const FVector AwayDir = (Delta2D / Dist2D);

        float W = 0.f;

        if (Dist2D < DesiredSep)
        {
            bAnyTrueOverlap = true;

            const float Pen = (DesiredSep - Dist2D); // world units
            if (Pen > StrongestPen)
            {
                StrongestPen = Pen;
                StrongestAway2D = AwayDir;
            }

            float t = Pen / FMath::Max(DesiredSep, 1.f);
            t = FMath::Clamp(t, 0.f, 1.f);
            MaxOverlapT = FMath::Max(MaxOverlapT, t);

            // Still strong, but slightly less "brick wall" than V9
            W = FMath::Pow(t, 1.35f) * 4.6f; // V9 was ~Pow 1.2 * 6.0
        }
        else
        {
            float t = (ReactDist - Dist2D) / FMath::Max(PreBuffer, 1.f);
            t = FMath::Clamp(t, 0.f, 1.f);
            MaxPreT = FMath::Max(MaxPreT, t);

            // Prebuffer guidance: slightly gentler to reduce twitch
            W = FMath::Pow(t, 1.0f) * 0.9f; // V9 was 1.2
        }

        SepSum2D += AwayDir * W;
        TotalW += W;
    }

    if (TotalW <= KINDA_SMALL_NUMBER || SepSum2D.IsNearlyZero()) return;

    FVector SepDir2D = (SepSum2D / TotalW).GetSafeNormal();

    // Never push backwards
    const float BackDot = FVector::DotProduct(SepDir2D, Forward2D);
    if (BackDot < 0.f)
    {
        SepDir2D -= Forward2D * BackDot;
        if (SepDir2D.IsNearlyZero()) return;
        SepDir2D = SepDir2D.GetSafeNormal();
    }

    // ---------------------------
    // Micro depenetration (softened)
    // Allow a tiny tolerated overlap before pushing out.
    // ---------------------------
    if (bAnyTrueOverlap && StrongestPen > 0.f && !StrongestAway2D.IsNearlyZero())
    {
        // Tolerate a tiny penetration ("95% hardness")
        const float PenTolerance = FMath::Max(1.5f, UnitRadius * 0.03f); // ~0.9 units for R=30, clamped up to 1.5
        const float ExcessPen = StrongestPen - PenTolerance;

        if (ExcessPen > 0.f)
        {
            const float PushFrac = 0.45f; // V9 was 0.60 (softer)
            const float MaxPush = FMath::Max(1.5f, UnitRadius * 0.14f); // V9 was 0.20 (softer cap)

            const float PushDist = FMath::Min(ExcessPen * PushFrac, MaxPush);
            const FVector PushDelta = FVector(StrongestAway2D.X, StrongestAway2D.Y, 0.f) * PushDist;

            FHitResult Hit;
            CapComp->MoveComponent(PushDelta, CapComp->GetComponentQuat(), true, &Hit);
        }
    }

    // ---------------------------
    // Steering (softened a bit)
    // ---------------------------
    float Blend = SoftCollisionBlend;

    if (bAnyTrueOverlap)
    {
        // not quite "absolute wall" anymore
        Blend = FMath::Max(Blend, 0.86f);     // V9 used ~0.92+
        Blend += MaxOverlapT * 0.06f;
    }
    else
    {
        Blend += MaxPreT * 0.20f;
        Blend = FMath::Clamp(Blend, 0.f, 0.65f);
    }

    Blend = FMath::Clamp(Blend, 0.f, 0.95f);

    FVector NewDir2D = (Forward2D * (1.f - Blend)) + (SepDir2D * Blend);
    if (NewDir2D.IsNearlyZero()) return;
    NewDir2D = NewDir2D.GetSafeNormal();

    // Final safety
    if (FVector::DotProduct(NewDir2D, Forward2D) <= 0.f)
    {
        NewDir2D = (SepDir2D + Forward2D * 0.10f).GetSafeNormal();
        if (NewDir2D.IsNearlyZero()) return;
    }

    InOutDesiredDir = FVector(NewDir2D.X, NewDir2D.Y, 0.f);

    if (bDebugSoftCollision)
    {
        const float Z = CapCenter.Z;
        DrawDebugCapsule(GetWorld(), CapCenter, CapHalf, QueryRadius, FQuat::Identity,
            FColor::Yellow, false, 0.05f, 0, 1.5f);

        DrawDebugLine(GetWorld(),
            FVector(CapCenter.X, CapCenter.Y, Z),
            FVector(CapCenter.X, CapCenter.Y, Z) + (SepDir2D * SoftCollisionDebugLen),
            FColor::Yellow, false, 0.05f, 0, 2.f);
    }
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