#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RobotStateVisualizer.generated.h"

class UWidgetComponent;
class UUserWidget;

UENUM(BlueprintType)
enum class ERobotVisualState : uint8
{
    Idle            UMETA(DisplayName = "Idle - No Task"),
    MovingToPickup  UMETA(DisplayName = "Moving to Pickup"),
    MovingToDropoff UMETA(DisplayName = "Moving to Dropoff"),
    Waiting         UMETA(DisplayName = "Waiting (MAPF)"),
    Conflict        UMETA(DisplayName = "Conflict / Replanning"),
    Charging        UMETA(DisplayName = "Charging"),
    GrabbingPallet  UMETA(DisplayName = "Grabbing Pallet"),
    PlacingPallet   UMETA(DisplayName = "Placing Pallet")
};

USTRUCT(BlueprintType)
struct FRobotStateInfo
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly)
    FText StateText;

    UPROPERTY(BlueprintReadOnly)
    FLinearColor StateColor;

    UPROPERTY(BlueprintReadOnly)
    UTexture2D* StateIcon = nullptr;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class PROJECTAI_WAREHOUSE_API URobotStateVisualizer : public UActorComponent
{
    GENERATED_BODY()

public:
    URobotStateVisualizer();

    UFUNCTION(BlueprintCallable, Category = "Robot Visualizer")
    void SetVisualState(ERobotVisualState NewState);

    UFUNCTION(BlueprintCallable, Category = "Robot Visualizer")
    void SetAgentName(const FString& Name);

    // Виджет для отображения
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Widget")
    TSubclassOf<UUserWidget> StatusWidgetClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Widget")
    FVector WidgetOffset = FVector(0, 0, 250.0f); // Высота над роботом

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Widget")
    bool bShowWidget = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bDebugLog = false;

    UFUNCTION(BlueprintCallable, Category = "Robot Visualizer")
    static void UpdateAgentState(AActor* Owner, ERobotVisualState NewState);

    UFUNCTION(BlueprintCallable)
    void SetHighlighted(bool bHighlighted);

    UPROPERTY(EditAnywhere, Category = "Highlight")
    FLinearColor HighlightColor = FLinearColor::Yellow;

protected:
    virtual void BeginPlay() override;

private:
    UPROPERTY()
    class UWidgetComponent* StatusWidgetComponent;

    UPROPERTY()
    class UUserWidget* StatusWidgetInstance;

    FString AgentName = TEXT("Робот ?");
    ERobotVisualState CurrentState = ERobotVisualState::Idle;

    void CreateStatusWidget();
    void UpdateStatusWidget();
    FRobotStateInfo GetStateInfo(ERobotVisualState State) const;
};