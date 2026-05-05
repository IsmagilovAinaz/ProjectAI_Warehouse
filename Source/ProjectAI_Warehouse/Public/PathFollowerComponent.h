#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PathFollowerComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class PROJECTAI_WAREHOUSE_API UPathFollowerComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UPathFollowerComponent();

    // === MAPF ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF")
    int32 AgentID = 0;

    // === ƒ¬»∆≈Õ»≈ ===
    UPROPERTY(EditAnywhere, Category = "Movement", meta = (ClampMin = "50", ClampMax = "500"))
    float Speed = 200.0f;

    UPROPERTY(EditAnywhere, Category = "Movement", meta = (ClampMin = "5", ClampMax = "50"))
    float ArrivalThreshold = 25.0f;

    UPROPERTY(EditAnywhere, Category = "Movement")
    float RotationSpeed = 10.0f;

    // === Œ“À¿ƒ ¿ ===
    UPROPERTY(EditAnywhere, Category = "Debug")
    bool bDrawDebugPath = true;

    UPROPERTY(EditAnywhere, Category = "Debug")
    float DebugDrawDuration = 5.0f;

    // === —Œ¡€“»ﬂ ===
    UFUNCTION(BlueprintCallable, Category = "Movement")
    void SetPath(const TArray<FVector>& Path);

    UFUNCTION(BlueprintCallable, Category = "Movement")
    bool IsPathComplete() const { return bFinished; }

    UFUNCTION(BlueprintCallable, Category = "Events")
    void OnPathFinished();

protected:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    TArray<FVector> Waypoints;
    int32 CurrentIndex = 0;
    bool bFinished = false;
    bool bReleased = false;

    void DrawDebugPath();
    void ClearDebugPath();
};