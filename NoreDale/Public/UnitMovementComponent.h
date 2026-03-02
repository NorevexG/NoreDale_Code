// UnitMovementComponent.h

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NavigationPath.h"
#include "UnitMovementComponent.generated.h"
class ACharacter;
class UCapsuleComponent;
class UCharacterMovementComponent;

UENUM(BlueprintType)
enum class ESteeringAuthority : uint8
{
    None                UMETA(DisplayName = "None"),
    PathFollowing       UMETA(DisplayName = "Path Following"),
    SoftCollision       UMETA(DisplayName = "Soft Collision")
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent), Blueprintable)
class NOREDALE_API UUnitMovementComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UUnitMovementComponent();

protected:
    virtual void BeginPlay() override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // ---------- Movement settings ----------
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float MoveSpeed = 400.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float AcceptanceModifier = 0.5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float RotationModifier = 30.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    float RotationSpeed = 600.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Movement")
    bool bUseNavMesh = true;

    // ---------- Debug / Avoidance detection (debug-only) ----------
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Debug")
    bool bDebugAvoidance = true;

    // How far ahead to probe
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Debug")
    float AvoidanceCheckDistance = 220.f;

    // Extra clearance buffer added to combined radii for "shoulder" checks
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Debug")
    float ExtraBuffer = 0.f;

    // Soft Collision
    UPROPERTY(EditAnywhere, Category = "Soft Collision")
    float SoftGateBuffer = 40.f;
    UPROPERTY(EditAnywhere, Category = "Soft Collision")
    bool bUnitInRangeForSoft = false;
    UPROPERTY(EditAnywhere, Category = "Soft Collision")
    bool bEnableSoftCollision = true;

    UPROPERTY(EditAnywhere, Category = "Soft Collision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SoftCollisionBlend = 0.35f; // how much we bend toward separation

    UPROPERTY(EditAnywhere, Category = "Soft Collision|Debug")
    bool bDebugSoftCollision = false;

    UPROPERTY(EditAnywhere, Category = "Soft Collision|Debug")
    float SoftCollisionDebugLen = 120.f;

    // ---------- Blueprint-callable API ----------
    UFUNCTION(BlueprintCallable, Category = "Movement")
    void RefreshMovementData();

    UFUNCTION(BlueprintCallable, Category = "Movement")
    TArray<FVector> MoveToLocation(const FVector& TargetLocation, bool bUseNav = true);

    UFUNCTION(BlueprintCallable, Category = "Movement")
    TArray<FVector> MoveToActor(AActor* TargetActor);

    UFUNCTION(BlueprintCallable, Category = "Movement")
    void SetPath(const TArray<FVector>& PathPoints, bool bStartMoving = true);

    UFUNCTION(BlueprintCallable, Category = "Movement")
    void StopMovement();

    UFUNCTION(BlueprintPure, Category = "Movement")
    bool IsMoving() const { return bMoving; }

protected:

    // ---------- Path Data ----------
    UPROPERTY(VisibleAnywhere, Category = "Steering")
    ESteeringAuthority SteeringAuthority = ESteeringAuthority::PathFollowing;

    UPROPERTY(Transient)
    TArray<FVector> InternalPathPoints;

    int8 AvoidanceSide = 0;

    int32 CurrentPathIndex = 0;
    bool bMoving = false;

    TWeakObjectPtr<AActor> MoveTargetActor;
    ACharacter* CharacterOwner = nullptr;

    UCapsuleComponent* CapComp = nullptr;
    UCharacterMovementComponent* MoveComp = nullptr;
    float UnitRadius = 30.f;
    float RotationSizeScale;

    FVector CurrentGoal = FVector::ZeroVector;
    //temp for soft collision
    TWeakObjectPtr<AActor> SoftCollisionActor;

    // ---------- Emergency detection (PURE DETECTION ONLY; no steering/execution) ----------
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    bool bEnableEmergencyDetection = true;

    // Visual indicator set by detection (ForwardColor will be forced red when true)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Emergency|Detect")
    bool bEmergencyDetected = false;

    // Deadlock: relative angle to blocker stays nearly constant for this long.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    float Emergency_DeadlockStableSec = 0.25f;

    // Angle delta below this is considered "stable" (radians).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    float Emergency_SmallAngleEpsRad = 0.01f;

    // Spin: cumulative signed orbit angle exceeds this (radians). Default = 2*pi.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    float Emergency_SpinOrbitRad = 6.28318530718f;

    // Flip-flop: sign toggles within a short time window.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    float Emergency_FlipWindowSec = 0.20f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    int32 Emergency_FlipToggleThreshold = 4;

    // Progress safety net (used only to validate Spin/Flip; Deadlock bypasses it).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    float Emergency_ProgressWindowSec = 0.25f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Emergency|Detect")
    float Emergency_MinProgressUnits = 5.f;

    // --- internal state (not gameplay execution) ---
    TWeakObjectPtr<AActor> Emergency_TrackedBlocker;
    bool Emergency_bHasPrevRelAngle = false;
    float Emergency_PrevRelAngleRad = 0.f;
    float Emergency_OrbitSignedRad = 0.f;
    float Emergency_AngleStableTime = 0.f;

    bool Emergency_bHasPrevPos2D = false;
    FVector2D Emergency_PrevPos2D = FVector2D::ZeroVector;

    int32 Emergency_PrevSideSign = 0;
    int32 Emergency_FlipToggleCount = 0;
    float Emergency_FlipWindowTime = 0.f;

    struct FEmergencyProgressSample
    {
        float Dt = 0.f;
        float Forward = 0.f;
    };
    TArray<FEmergencyProgressSample> Emergency_ProgressSamples;
    float Emergency_ProgressWindowTime = 0.f;
    float Emergency_ProgressWindowSum = 0.f;

    void ResetEmergencyDetection();
    void UpdateEmergencyDetection(const FVector& DesiredDir, const FVector& FinalDir, const FVector& MyPos, float DeltaTime, bool bSoftCollisionActive);

    //Debug Variables
    FColor ForwardColor;

    //DebugTools
    void DebugTools(const FVector& FinalDir, const FVector& MyPos);

    // Build navigation path
    bool BuildPathToLocation(const FVector& TargetLocation);

    // Main movement tick
    bool TickMoveAlongPath(float DeltaTime);

    // Debug-only detection (no steering)
    bool AvoidanceCheck(FVector& DesiredDir, const FVector& MyPos, float DeltaTime);

    //SoftCollision
    bool SoftCollision(const FVector& ForwardDir, const FVector& MyPos, float DeltaTime, FVector& OutMoveDir2D);

    //GoalAdjustments
    bool GoalAdjustment();

    //ClippingFix
    bool ClippingFix(const FVector& MyPos);

    //ComputeSafestPoint
    bool ComputeSafestPoint(TArray<FOverlapResult>& NearObjects);

    // Rotation helper
    void SmoothFaceDirection(const FVector& Direction, float DeltaTime);

    //Helper Functions:
    //SetOtherRadius
    float SetOtherRadius(const AActor* Other);
};
