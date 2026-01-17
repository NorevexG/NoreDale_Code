// UnitMovementComponent.h

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NavigationPath.h"
#include "UnitMovementComponent.generated.h"
class ACharacter;
class UCapsuleComponent;
class UCharacterMovementComponent;

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

    //temp for version 5 avoidance
    FVector PrevAvoidDir2D = FVector::ZeroVector;

    // Build navigation path
    bool BuildPathToLocation(const FVector& TargetLocation);

    // Main movement tick
    bool TickMoveAlongPath(float DeltaTime);

    // Debug-only detection (no steering)
    bool AvoidanceCheck(FVector& DesiredDir, const FVector& MyPos, float DeltaTime);

    //ApplyAvoidance
    void ApplyAvoidance(FVector& DesiredDir, const FVector& MyPos, TArray<AActor*> NearActors);

    void OldApplyAvoidance(FVector& DesiredDir, const FVector& MyPos, const FVector& OtherPos, float OtherRadius);

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