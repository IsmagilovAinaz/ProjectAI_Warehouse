// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WarehouseTypes.generated.h"

UENUM(BlueprintType)
enum class EAllocationStrategy : uint8
{
	Greedy      UMETA(DisplayName = "Greedy (Nearest)"),
	Auction     UMETA(DisplayName = "Auction (Cost-Based)")
};

class PROJECTAI_WAREHOUSE_API WarehouseTypes
{
public:
	WarehouseTypes();
	~WarehouseTypes();
};
