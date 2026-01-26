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
    None            UMETA(DisplayName = "None"),
    PathFollowing   UMETA(DisplayName = "Path Following"),
    InitialAvoidance UMETA(DisplayName = "Initial Avoidance"),
    SoftCollision   UMETA(DisplayName = "Soft Collision")
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

    //temp for version 5-9 apply avoidance
    FVector PrevAvoidDir2D = FVector::ZeroVector;


    //temp AvoidanceCheck v6
    float Avoid_InitialLockRemaining = 0.f;
    TWeakObjectPtr<AActor> Avoid_LockedDominant;
    float Avoid_RecentBlockRemaining = 0.f;


    //Temp New InitialAvoidance V2
    TWeakObjectPtr<AActor> IA_Target;
    int32 IA_SideSign = 0;
    float IA_MinDist = BIG_NUMBER;
    // v4 additions
    FVector IA_BaseDir2D = FVector::ZeroVector;  
    bool IA_bReleasing = false;
    float IA_ReleaseAlpha = 1.f;
    FVector IA_LastAvoidDir2D = FVector::ZeroVector;

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

    //InitialAvoidance
    void InitialAvoidance(FVector& DesiredDir, const FVector& MyPos, AActor* DominantBlocker);

    //EmergancyAvoidance

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