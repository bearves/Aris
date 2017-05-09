#ifndef IO_MAPPING_DEFINITION_H

class MappingDefinition
{
public:
	int targetPosition_index = 0;
	int targetPosition_subindex = 0;
	int targetVelocity_index = 0;
	int targetVelocity_subindex = 1;
	int targetTorque_index = 0;
	int targetTorque_subindex = 2;
	int controlWord_index = 0;
	int controlWord_subindex = 4;
	int modeOfOperation_index = 0;
	int modeOfOperation_subindex = 5;
	int positionActualValue_index = 1;
	int positionActualValue_subindex = 0;
	int torqueActualValue_index = 1;
	int torqueActualValue_subindex = 2;
	int statusWord_index = 1;
	int statusWord_subindex = 3;
	int modeOfOperationDisplay_index = 1;
	int modeOfOperationDisplay_subindex = 4;
	int velocityActualValue_index = 2;
	int velocityActualValue_subindex = 0;
	int digitalInputs_index = 3;
	int digitalInputs_subindex = 0;
	int home_count_sdo_index = 9;
};

class ElmoGuitarDef : public MappingDefinition
{
public:
	ElmoGuitarDef()
	{
		targetPosition_index = 0;
		targetPosition_subindex = 0;
		targetVelocity_index = 0;
		targetVelocity_subindex = 1;
		targetTorque_index = 0;
		targetTorque_subindex = 2;
		controlWord_index = 0;
		controlWord_subindex = 4;
		modeOfOperation_index = 0;
		modeOfOperation_subindex = 5;
		positionActualValue_index = 1;
		positionActualValue_subindex = 0;
		torqueActualValue_index = 1;
		torqueActualValue_subindex = 2;
		statusWord_index = 1;
		statusWord_subindex = 3;
		modeOfOperationDisplay_index = 1;
		modeOfOperationDisplay_subindex = 4;
		velocityActualValue_index = 2;
		velocityActualValue_subindex = 0;
		digitalInputs_index = 3;
		digitalInputs_subindex = 0;
		home_count_sdo_index = 9;
	}
};

class FaulhaberDef : public MappingDefinition
{
public:
	FaulhaberDef()
	{
		targetPosition_index = 0;
		targetPosition_subindex = 0;
		targetVelocity_index = 0;
		targetVelocity_subindex = 1;
		targetTorque_index = 0;
		targetTorque_subindex = 2;
		controlWord_index = 0;
		controlWord_subindex = 3;

		modeOfOperation_index = 1;
		modeOfOperation_subindex = 0;

		positionActualValue_index = 2;
		positionActualValue_subindex = 0;
		torqueActualValue_index = 2;
		torqueActualValue_subindex = 1;
		statusWord_index = 2;
		statusWord_subindex = 2;
		modeOfOperationDisplay_index = 2;
		modeOfOperationDisplay_subindex = 3;
		velocityActualValue_index = 3;
		velocityActualValue_subindex = 0;
		digitalInputs_index = 3;
		digitalInputs_subindex = 0;
		home_count_sdo_index = 7;
	}
};

class CopleyAE2Def_A : public MappingDefinition
{
public:
	CopleyAE2Def_A()
	{
		targetPosition_index = 0;
		targetPosition_subindex = 0;
		targetVelocity_index = 0;
		targetVelocity_subindex = 1;
		targetTorque_index = 0;
		targetTorque_subindex = 2;
		controlWord_index = 0;
		controlWord_subindex = 3;

		modeOfOperation_index = 1;
		modeOfOperation_subindex = 0;

		positionActualValue_index = 2;
		positionActualValue_subindex = 0;
		torqueActualValue_index = 2;
		torqueActualValue_subindex = 1;
		statusWord_index = 2;
		statusWord_subindex = 2;
		modeOfOperationDisplay_index = 2;
		modeOfOperationDisplay_subindex = 3;

		velocityActualValue_index = 3;
		velocityActualValue_subindex = 0;
		digitalInputs_index = 3;
		digitalInputs_subindex = 0;

		home_count_sdo_index = 5;
	}
};

class CopleyAE2Def_B : public MappingDefinition
{
public:
	CopleyAE2Def_B()
	{
		targetPosition_index = 4;
		targetPosition_subindex = 0;
		targetVelocity_index = 4;
		targetVelocity_subindex = 1;
		targetTorque_index = 4;
		targetTorque_subindex = 2;
		controlWord_index = 4;
		controlWord_subindex = 3;

		modeOfOperation_index = 5;
		modeOfOperation_subindex = 0;

		positionActualValue_index = 6;
		positionActualValue_subindex = 0;
		torqueActualValue_index = 6;
		torqueActualValue_subindex = 1;
		statusWord_index = 6;
		statusWord_subindex = 2;
		modeOfOperationDisplay_index = 6;
		modeOfOperationDisplay_subindex = 3;

		velocityActualValue_index = 7;
		velocityActualValue_subindex = 0;
		digitalInputs_index = 7;
		digitalInputs_subindex = 0;

		home_count_sdo_index = 11;
	}
};
#endif // !IO_MAPPING_DEFINITION_H
