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

// Reworked Version 6
bool UUnitMovementComponent::AvoidanceCheck(FVector& DesiredDir, const FVector& MyPos, float DeltaTime)
{
    bool bGoalAdjusted = false;
    if (!CharacterOwner) return bGoalAdjusted;

    // --- setup ---
    FVector ToGoal = CurrentGoal - MyPos;
    FVector ToGoal2D(ToGoal.X, ToGoal.Y, 0.f);
    float DistToGoal = ToGoal2D.Size();
    float Range = FMath::Max(50.f, AvoidanceCheckDistance);
    if (DistToGoal < KINDA_SMALL_NUMBER) return bGoalAdjusted;

    const bool bIsFinalPoint = (InternalPathPoints.Num() > 0) && (CurrentPathIndex >= InternalPathPoints.Num() - 1);

    // --- build bent corridor (A then spill into B) ---
    const FVector MyPos2D(MyPos.X, MyPos.Y, 0.f);

    // Point A: current path point if valid, else CurrentGoal
    FVector PointA3D = CurrentGoal;
    if (InternalPathPoints.IsValidIndex(CurrentPathIndex))
        PointA3D = InternalPathPoints[CurrentPathIndex];

    const FVector PointA2D(PointA3D.X, PointA3D.Y, 0.f);

    FVector DirA2D = (PointA2D - MyPos2D);
    float DistToA = DirA2D.Size();
    DirA2D = (DistToA > KINDA_SMALL_NUMBER) ? (DirA2D / DistToA) : FVector::ZeroVector;

    // IMPORTANT FIX:
    // If we're on the final path point, ensure segment A reaches the goal.
    // Otherwise we can miss "goal near/inside another unit" cases.
    float LenA = bIsFinalPoint ? DistToA : FMath::Min(Range, DistToA);

    FVector StartA = MyPos2D;
    FVector EndA = MyPos2D + DirA2D * LenA;

    // Keep Remaining consistent with overlap radius (Range), even if LenA > Range on final point
    float Remaining = Range - FMath::Min(Range, LenA);

    // Point B: next path point if valid
    float LenB = 0.f;
    FVector StartB = PointA2D;
    FVector EndB = PointA2D;
    FVector DirB2D = FVector::ZeroVector;

    if (Remaining > KINDA_SMALL_NUMBER && InternalPathPoints.IsValidIndex(CurrentPathIndex + 1))
    {
        FVector PointB3D = InternalPathPoints[CurrentPathIndex + 1];
        FVector PointB2D(PointB3D.X, PointB3D.Y, 0.f);

        FVector AB = (PointB2D - PointA2D);
        float DistAB = AB.Size();
        if (DistAB > KINDA_SMALL_NUMBER)
        {
            DirB2D = AB / DistAB;
            LenB = FMath::Min(Remaining, DistAB);
            StartB = PointA2D;
            EndB = PointA2D + DirB2D * LenB;
        }
    }

    // Debug line end (last segment end) - not used for drawing (you draw constant length)
    FVector ForwardEnd = (LenB > 0.f) ? EndB : EndA;

    // Detection state
    bool bDetectedAny = false; // FIX: any valid unit in overlap -> BLUE
    bool bBlockingPath = false;
    FColor ForwardColor = FColor::Green;

    // --- overlap query (broadphase) ---
    TArray<FOverlapResult> Overlaps;
    FCollisionObjectQueryParams ObjQuery;
    ObjQuery.AddObjectTypesToQuery(ECC_Pawn);
    ObjQuery.AddObjectTypesToQuery(ECC_PhysicsBody);

    FCollisionQueryParams Params(SCENE_QUERY_STAT(AvoidanceOverlap), true, CharacterOwner);

    bool bAny = GetWorld()->OverlapMultiByObjectType(
        Overlaps,
        MyPos,
        FQuat::Identity,
        ObjQuery,
        FCollisionShape::MakeSphere(Range),
        Params
    );

    TArray<AActor*> BlockingActors;

    // --- helpers ---
    auto ClosestPointOnSegment2D = [&](const FVector& P, const FVector& S, const FVector& E, float& OutAlong, FVector& OutClosest) -> void
    {
        FVector SE = (E - S);
        float SegLen = SE.Size();
        if (SegLen < KINDA_SMALL_NUMBER)
        {
            OutAlong = 0.f;
            OutClosest = S;
            return;
        }

        FVector Dir = SE / SegLen;
        float t = FVector::DotProduct((P - S), Dir); // distance-along in world units
        t = FMath::Clamp(t, 0.f, SegLen);

        OutAlong = t;
        OutClosest = S + Dir * t;
    };

    // Side detection removed: this is now ONLY hard corridor intersection.
    auto TestSegment = [&](AActor* Other,
        const FVector& SegStart,
        const FVector& SegEnd,
        float SegLen,
        float CombinedRadius,
        bool& bOutInFront) -> bool
    {
        bOutInFront = false;

        if (SegLen < KINDA_SMALL_NUMBER)
            return false;

        const FVector OtherLoc = Other->GetActorLocation();
        const FVector Other2D(OtherLoc.X, OtherLoc.Y, 0.f);

        float Along = 0.f;
        FVector Closest = SegStart;
        ClosestPointOnSegment2D(Other2D, SegStart, SegEnd, Along, Closest);

        // In-front means projection is somewhere on the segment AND not behind start
        FVector SegDir = (SegEnd - SegStart).GetSafeNormal();
        float RawAlong = FVector::DotProduct((Other2D - SegStart), SegDir);
        if (RawAlong > 0.f && RawAlong <= SegLen + 1.f)
            bOutInFront = true;

        FVector Delta = Other2D - Closest;
        float PerpDist = Delta.Size();

        // Hard blocking: inside corridor radius and in-front
        if (PerpDist <= CombinedRadius && bOutInFront)
            return true;

        return false;
    };

    // --- main loop ---
    if (bAny && Overlaps.Num() > 0)
    {
        for (const FOverlapResult& R : Overlaps)
        {
            AActor* Other = R.GetActor();
            if (!Other || Other == CharacterOwner) continue;
            if (MoveTargetActor.IsValid() && Other == MoveTargetActor.Get()) continue;

            const FVector OtherLoc = Other->GetActorLocation();
            const FVector ToOther2D(OtherLoc.X - MyPos.X, OtherLoc.Y - MyPos.Y, 0.f);
            if (ToOther2D.SizeSquared() < 1.f) continue;

            // "detected something" is true as soon as we accept a valid Other (blue)
            bDetectedAny = true;

            float OtherRadius = SetOtherRadius(Other);
            float CombinedRadius = UnitRadius + OtherRadius + ExtraBuffer;

            // Final-goal proximity blocking (kept)
            if (bIsFinalPoint)
            {
                const FVector Other2D(OtherLoc.X, OtherLoc.Y, 0.f);
                const float GoalDist = (PointA2D - Other2D).Size();

                if (GoalDist <= CombinedRadius)
                {
                    BlockingActors.AddUnique(Other);
                    continue;
                }
            }

            bool bInFrontA = false;
            bool bInFrontB = false;

            bool bBlockA = TestSegment(Other, StartA, EndA, LenA, CombinedRadius, bInFrontA);
            bool bBlockB = (LenB > 0.f) ? TestSegment(Other, StartB, EndB, LenB, CombinedRadius, bInFrontB) : false;

            if (bBlockA || bBlockB)
            {
                BlockingActors.AddUnique(Other);
            }
        }

        if (BlockingActors.Num() > 0)
        {
            if (CurrentPathIndex >= InternalPathPoints.Num() - 1)
            {
                bGoalAdjusted = GoalAdjustment();
                if (bGoalAdjusted)
                {
                    return bGoalAdjusted;
                }
            }

            bBlockingPath = true;
            //ApplyAvoidance(DesiredDir, MyPos, BlockingActors);
        }
    }

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

                // draw hard red capsule and line indicating clipping
                DrawDebugCapsule(GetWorld(), CapCenter, CapHalfHeight, CapRadius, FQuat::Identity, FColor::Red, false, 0.05f, 0, 2.f);
                DrawDebugLine(GetWorld(), MyPos, Other->GetActorLocation(), FColor::Red, false, 0.05f, 0, 3.f);
            }
        }
    }

    // Blue/Red/Green as requested
    if (bBlockingPath) ForwardColor = FColor::Red;
    else if (bDetectedAny) ForwardColor = FColor::Blue;
    else ForwardColor = FColor::Green;

    if (bDebugAvoidance)
    {
        float Z = CharacterOwner->GetActorLocation().Z;
        if (CapComp)
        {
            Z += CapComp->GetScaledCapsuleHalfHeight() * 0.5f;
        }

        FVector Start = MyPos;
        Start.Z = Z;

        FVector Dir = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
        FVector End = Start + Dir * Range;

        DrawDebugLine(GetWorld(), Start, End, ForwardColor, false, 0.05f, 0, 4.f);
    }

    /*-----------------------------------------DEBUG----------------------------------------------------*/

    return bGoalAdjusted;
}

//ApplyAvoidance
//Version 9
void UUnitMovementComponent::ApplyAvoidance(FVector& DesiredDir, const FVector& MyPos, TArray<AActor*> NearActors)
{
    // --- 2D only ---
    FVector ForwardDir = FVector(DesiredDir.X, DesiredDir.Y, 0.f).GetSafeNormal();
    if (ForwardDir.IsNearlyZero() || !CharacterOwner || !MoveComp) return;

    // -------------------------
    // FINAL GOAL GUARD (IMPORTANT)
    // -------------------------
    // When we're navigating to the FINAL path point, avoidance should not "pass around" and cause orbiting.
    // Let GoalAdjustment/ComputeSafestPoint handle occupied destination.
    const bool bHasPath = (InternalPathPoints.Num() > 0);
    const bool bIsFinalPoint = bHasPath && (CurrentPathIndex >= InternalPathPoints.Num() - 1);

    float DistToFinal2D = 0.f;
    if (bIsFinalPoint)
    {
        const FVector ToFinal = InternalPathPoints.Last() - MyPos;
        DistToFinal2D = FVector(ToFinal.X, ToFinal.Y, 0.f).Size();
    }

    // If we're close to the final destination, disable lateral avoidance completely.
    // Only allow separation if we are actually overlapping.
    const float FinalGuardRadius = FMath::Max(80.f, UnitRadius * 2.25f);
    const bool bFinalGuard = bIsFinalPoint && (DistToFinal2D > 0.f) && (DistToFinal2D <= FinalGuardRadius);

    // -------------------------
    // Stable velocities / horizon
    // -------------------------
    FVector MyVel2D = CharacterOwner->GetVelocity();
    MyVel2D.Z = 0.f;
    if (MyVel2D.IsNearlyZero())
    {
        MyVel2D = ForwardDir * FMath::Max(1.f, MoveSpeed);
    }

    const float Speed = FMath::Max(1.f, MoveSpeed);
    const float SpeedFactor = FMath::Clamp(Speed / 600.f, 0.5f, 2.0f);
    const float Horizon = FMath::Clamp(0.25f + 0.35f * SpeedFactor, 0.25f, 0.90f);

    // -------------------------
    // File-static avoidance state (no header changes)
    // -------------------------
    struct FAvoidState
    {
        TWeakObjectPtr<AActor> Threat;
        float UntilTime = 0.f;
        int32 Side = 0;           // -1 or +1, committed
        float YieldUntilTime = 0.f;
        FVector LastPos = FVector::ZeroVector;
        float LastGoalDist = 0.f;
        float StuckTime = 0.f;
    };

    static TMap<TWeakObjectPtr<const UUnitMovementComponent>, FAvoidState> StateMap;
    FAvoidState& S = StateMap.FindOrAdd(this);

    const float Now = (GetWorld() ? GetWorld()->GetTimeSeconds() : 0.f);

    // Update stuck/progress tracker (uses final point if present, otherwise current goal)
    {
        FVector Goal = bHasPath ? InternalPathPoints[CurrentPathIndex] : CurrentGoal;
        const float GoalDist = FVector::Dist2D(MyPos, Goal);

        if (!S.LastPos.IsNearlyZero())
        {
            const float Moved = FVector::Dist2D(MyPos, S.LastPos);
            const bool bProgress = (GoalDist + 1.0f < S.LastGoalDist); // got closer

            // "stuck" if we are barely moving AND not progressing
            if (Moved < 2.0f && !bProgress)
                S.StuckTime += GetWorld()->GetDeltaSeconds();
            else
                S.StuckTime = FMath::Max(0.f, S.StuckTime - GetWorld()->GetDeltaSeconds() * 2.0f);
        }

        S.LastPos = MyPos;
        S.LastGoalDist = GoalDist;
    }

    // If we are currently in a yield window, reduce forward drive (lets the other pass)
    const bool bYielding = (Now < S.YieldUntilTime);

    // -------------------------
    // Find primary threat + accumulate separation
    // -------------------------
    AActor* Primary = nullptr;
    float PrimaryScore = -FLT_MAX;

    // We only accumulate separation in final-guard mode.
    FVector AccumSide = FVector::ZeroVector;
    FVector AccumSep = FVector::ZeroVector;
    float TotalSideW = 0.f;
    float TotalSepW = 0.f;

    for (AActor* Other : NearActors)
    {
        if (!Other || Other == CharacterOwner) continue;
        if (MoveTargetActor.IsValid() && Other == MoveTargetActor.Get()) continue;

        const FVector OtherPos = Other->GetActorLocation();
        const FVector RelPos2D = FVector(OtherPos.X - MyPos.X, OtherPos.Y - MyPos.Y, 0.f);

        const float DistNow = RelPos2D.Size();
        if (DistNow < KINDA_SMALL_NUMBER) continue;

        const FVector DirToOtherNow = RelPos2D / DistNow;

        // Keep loose front filter so X crossing still triggers
        const float ForwardDot = FVector::DotProduct(ForwardDir, DirToOtherNow);
        if (ForwardDot < -0.25f) continue;

        const float OtherRadius = SetOtherRadius(Other);
        const float CombinedRadius = UnitRadius + OtherRadius + ExtraBuffer;

        FVector OtherVel2D = Other->GetVelocity();
        OtherVel2D.Z = 0.f;
        if (OtherVel2D.IsNearlyZero())
        {
            if (ACharacter* OC = Cast<ACharacter>(Other))
            {
                if (UCharacterMovementComponent* OMove = OC->GetCharacterMovement())
                {
                    OtherVel2D = (-DirToOtherNow) * FMath::Max(1.f, OMove->MaxWalkSpeed);
                }
            }
        }

        const FVector RelVel2D = OtherVel2D - MyVel2D;
        const float RelVelSq = RelVel2D.SizeSquared();

        float tClosest = 0.f;
        FVector ClosestVec = RelPos2D;
        if (RelVelSq > 25.f)
        {
            tClosest = -FVector::DotProduct(RelPos2D, RelVel2D) / RelVelSq;
            tClosest = FMath::Clamp(tClosest, 0.f, Horizon);
            ClosestVec = RelPos2D + RelVel2D * tClosest;
        }

        const float ClosestDist = ClosestVec.Size();

        const float LookAhead = (CombinedRadius * (3.2f * SpeedFactor)) + (Speed * Horizon);

        const float EdgeNow = DistNow - CombinedRadius;
        const float EdgePred = ClosestDist - CombinedRadius;
        const float EdgeDist = FMath::Min(EdgeNow, EdgePred);

        if (EdgeDist > LookAhead) continue;

        float Proximity = 1.f - FMath::Clamp(EdgeDist / LookAhead, 0.f, 1.f);
        Proximity = FMath::Pow(Proximity, 1.35f);

        const bool bOverlapping = (EdgeNow < 0.f);
        const bool bVeryClose = (EdgeNow < CombinedRadius * 0.45f);

        // Separation: stronger when close/overlapping. This is what prevents "push through."
        float SepW = 0.f;
        if (bVeryClose)
        {
            const float CloseAlpha = 1.f - FMath::Clamp(EdgeNow / FMath::Max(CombinedRadius * 0.45f, 1.f), 0.f, 1.f);
            SepW = FMath::Clamp(0.40f + CloseAlpha * 1.05f, 0.f, 1.45f);
        }
        if (bOverlapping) SepW = FMath::Max(SepW, 1.30f);

        const FVector SepDir = (-DirToOtherNow).GetSafeNormal();
        AccumSep += SepDir * SepW;
        TotalSepW += SepW;

        // Side passing (disabled in final-guard mode)
        if (!bFinalGuard)
        {
            // We do NOT re-decide side each tick. We use committed side (below).
            // Here we just measure "how much we want to pass" in general.
            const float SideW = FMath::Clamp(Proximity, 0.f, 1.25f);
            TotalSideW += SideW;
        }

        // Primary threat scoring (close + in front)
        const float Score = (Proximity * 1.15f + SepW * 1.0f) * FMath::Clamp(ForwardDot + 0.25f, 0.f, 1.f);
        if (Score > PrimaryScore)
        {
            PrimaryScore = Score;
            Primary = Other;
        }
    }

    // If nothing nearby, keep going (but allow a short "linger" commitment to avoid flip-flop)
    const bool bHasThreat = (Primary != nullptr);

    // -------------------------
    // Commit to a side (HARD) to stop mirror wiggle
    // -------------------------
    const bool bCommitValid = (S.Side != 0 && S.Threat.IsValid() && Now < S.UntilTime);

    if (!bFinalGuard && bHasThreat)
    {
        // Refresh commitment if threat changed or expired
        if (!bCommitValid || S.Threat.Get() != Primary)
        {
            S.Threat = Primary;

            // IMPORTANT: side is *stable per pair*, not based on noisy cross products.
            // Both units compute the same pair sign; because their ForwardDir is opposite, "my left" becomes opposite world directions.
            const int32 A = CharacterOwner->GetUniqueID();
            const int32 B = Primary->GetUniqueID();
            const int32 MinID = FMath::Min(A, B);
            const int32 MaxID = FMath::Max(A, B);

            // Stable pseudo-random bit from the pair (does not change frame-to-frame)
            const uint32 PairHash = HashCombineFast((uint32)MinID, (uint32)MaxID);
            S.Side = ((PairHash & 1u) == 0u) ? +1 : -1;

            // Commit longer to prevent flip-flop
            S.UntilTime = Now + FMath::Clamp(0.55f + 0.20f * SpeedFactor, 0.55f, 1.05f);
        }
    }
    else
    {
        // If no threat, let commitment decay naturally (don’t instantly drop to 0 or you’ll oscillate).
    }

    // -------------------------
    // Deadlock breaker (the real fix for infinite mirror-lock)
    // If we're stuck for long enough near a threat, force ONE unit to yield briefly.
    // -------------------------
    if (S.StuckTime > 0.35f && bHasThreat)
    {
        // Deterministic: higher ID yields (so only one yields)
        const int32 MeID = CharacterOwner->GetUniqueID();
        const int32 OtherID = Primary->GetUniqueID();

        if (MeID > OtherID)
        {
            S.YieldUntilTime = Now + 0.35f;   // short yield window
        }

        // Reset stuck time so we don't spam yield forever
        S.StuckTime = 0.f;
    }

    // -------------------------
    // Build final steering vector
    // -------------------------
    FVector SepN = AccumSep.IsNearlyZero() ? FVector::ZeroVector : AccumSep.GetSafeNormal();

    // If final-guard: do NOT do lateral passing (prevents orbiting around occupied goal)
    FVector SideN = FVector::ZeroVector;
    float SideBlend = 0.f;

    if (!bFinalGuard && (bCommitValid || (S.Side != 0 && Now < S.UntilTime)))
    {
        SideN = (S.Side > 0)
            ? FVector(ForwardDir.Y, -ForwardDir.X, 0.f)
            : FVector(-ForwardDir.Y, ForwardDir.X, 0.f);
        SideN.Normalize();

        SideBlend = FMath::Clamp(TotalSideW, 0.f, 1.f);

        // If yielding, we *don’t* try to pass; we mostly step aside (lets other go through)
        if (bYielding)
        {
            SideBlend = FMath::Max(SideBlend, 0.80f);
        }
    }

    float SepBlend = FMath::Clamp(TotalSepW, 0.f, 1.f);

    // Forward damping:
    // - If very close, separation dominates and forward drops so we stop "pushing through".
    // - If yielding, forward drops even more.
    float ForwardBlend = 1.f - FMath::Clamp(SepBlend * 0.85f + SideBlend * 0.70f, 0.f, bYielding ? 0.98f : 0.92f);

    FVector NewDir = ForwardDir * ForwardBlend;
    if (!SideN.IsNearlyZero()) NewDir += SideN * SideBlend;
    if (!SepN.IsNearlyZero())  NewDir += SepN * SepBlend;

    FVector FinalDir = NewDir.GetSafeNormal();
    if (FinalDir.IsNearlyZero()) return;

    // No backwards
    if (FVector::DotProduct(FinalDir, ForwardDir) <= 0.f)
    {
        // prefer sideways/separation fallback
        if (!SepN.IsNearlyZero()) FinalDir = (SepN + ForwardDir * 0.10f).GetSafeNormal();
        else if (!SideN.IsNearlyZero()) FinalDir = (SideN + ForwardDir * 0.10f).GetSafeNormal();
        else return;
    }

    // Less smoothing to avoid "synchronized oscillation"
    if (!PrevAvoidDir2D.IsNearlyZero())
    {
        FinalDir = (PrevAvoidDir2D * 0.45f + FinalDir * 0.55f).GetSafeNormal();
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