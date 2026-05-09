#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BehaviorTree/BehaviorTreeTypes.h"
#include "BTTask_RequestMAPFPath.generated.h"

// Enum должен быть объявлен где-то в вашем проекте, например в отдельном файле
// Если уже есть - удалите этот блок и подключите нужный заголовочный файл
UENUM(BlueprintType)
enum class E_TaskState : uint8
{
    MovingToPallet = 0  UMETA(DisplayName = "Moving To Pallet"),
    MovingToShelf = 1  UMETA(DisplayName = "Moving To Shelf"),
    Idle = 2  UMETA(DisplayName = "Idle"),
    TakePallet = 3  UMETA(DisplayName = "Take Pallet"),
    Rotating = 4  UMETA(DisplayName = "Rotating"),
    PlacePallet = 5  UMETA(DisplayName = "Place Pallet"),
    ChangeHeight = 6  UMETA(DisplayName = "Change Height"),
    FinishTask = 7  UMETA(DisplayName = "Finish Task")
};

UCLASS()
class PROJECTAI_WAREHOUSE_API UBTTask_RequestMAPFPath : public UBTTaskNode
{
    GENERATED_BODY()

public:
    UBTTask_RequestMAPFPath();

    virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
    virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;

    // ======== ПАРАМЕТРЫ КАК В MOVETOTASK ========

    // Следующее состояние (выпадающий список E_TaskState)
    UPROPERTY(EditAnywhere, Category = "Task")
    E_TaskState NextState;

    // Текущее состояние задачи (ключ Blackboard типа Enum)
    UPROPERTY(EditAnywhere, Category = "Blackboard")
    FBlackboardKeySelector TaskState;

    // ======== ПАРАМЕТРЫ MAPF ========

    // Целевая локация
    UPROPERTY(EditAnywhere, Category = "Blackboard")
    FBlackboardKeySelector GoalLocationKey;

    // ID агента
    UPROPERTY(EditAnywhere, Category = "Blackboard")
    FBlackboardKeySelector AgentIDKey;

    // Таймаут планирования (секунды)
    UPROPERTY(EditAnywhere, Category = "Planning")
    float PlanningTimeout = 30.0f;

private:
    // Состояние выполнения
    bool bIsRunning;
    float ElapsedTime;
    int32 CurrentAgentID;

    // Безопасное получение имени ключа
    FName GetKeyName(const FBlackboardKeySelector& Key) const
    {
        return Key.SelectedKeyName;
    }
};