#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "ClickableRobotInterface.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnRobotClickedSignature, AActor*, ClickedActor);

UINTERFACE(MinimalAPI, Blueprintable)
class UClickableRobotInterface : public UInterface
{
    GENERATED_BODY()
};

class PROJECTAI_WAREHOUSE_API IClickableRobotInterface
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintNativeEvent, Category = "Interaction")
    void OnRobotInteracted();
};