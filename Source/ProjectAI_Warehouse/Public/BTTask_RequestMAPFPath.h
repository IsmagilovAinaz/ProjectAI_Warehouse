#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTTaskNode.h"
#include "BTTask_RequestMAPFPath.generated.h"

UCLASS()
class PROJECTAI_WAREHOUSE_API UBTTask_RequestMAPFPath : public UBTTaskNode
{
    GENERATED_BODY()

public:
    UBTTask_RequestMAPFPath();

    // === BLACKBOARD KEYS ===
    UPROPERTY(EditAnywhere, Category = "MAPF")
    FName GoalLocationKey = "TargetLocation";

    UPROPERTY(EditAnywhere, Category = "MAPF")
    FName AgentIDKey = "AgentID";

    UPROPERTY(EditAnywhere, Category = "MAPF")
    float PlanningTimeout = 3.0f;

    // === EXECUTION ===
    virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
    virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;

private:
    bool bIsRunning = false;
    float ElapsedTime = 0.0f;
    int32 CurrentAgentID = -1;
};