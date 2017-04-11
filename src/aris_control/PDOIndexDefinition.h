#ifndef PDOINDEXDEFINITION_H

class ElmoGuitarPDOS
{
public:
	static const int targetPosition_index = 0;
	static const int targetPosition_subindex = 0;
	static const int targetVelocity_index = 0;
	static const int targetVelocity_subindex = 1;
	static const int targetTorque_index = 0;
	static const int targetTorque_subindex = 2;
	static const int maxTorque_index = 0;
	static const int maxTorque_subindex = 3;
	static const int controlWord_index = 0;
	static const int controlWord_subindex = 4;
	static const int modeOfOperation_index = 0;
	static const int modeOfOperation_subindex = 5;
	static const int positionActualValue_index = 1;
	static const int positionActualValue_subindex = 0;
	static const int positionFollowingErrorActualValue_index = 1;
	static const int positionFollowingErrorActualValue_subindex = 1;
	static const int torqueActualValue_index = 1;
	static const int torqueActualValue_subindex = 2;
	static const int statusWord_index = 1;
	static const int statusword_subindex = 3;
	static const int modeOfOperationDisplay_index = 1;
	static const int modeOfOperationDisplay_subindex = 4;
	static const int velocityActualValue_index = 2;
	static const int velocityActualValue_subindex = 0;
	static const int digitalInputs_index = 3;
	static const int digitalInputs_subindex = 0;
};

#endif // !PDOIndexDefinition_H
