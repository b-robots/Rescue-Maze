/*
This private part of the Library is responsible for the communication with the RasPI for the camera recognition.
*/

#include "../header/CamRec.h"

namespace JAFD
{
	namespace CamRec
	{
		namespace
		{
			VisVictimProb leftProb;
			VisVictimProb rightProb;
			uint16_t leftDet = 0;
			uint16_t rightDet = 0;
		}

		ReturnCode setup()
		{
			Serial1.begin(9600);

			Serial1.write("B");

			if (Serial1.find("OK\n"))
			{
				return ReturnCode::ok;
			}
			else
			{
				return ReturnCode::fatalError;
			}
		}

		void loop()
		{
			auto start = millis();

			// Wait for at least one full package
			while (true)
			{
				if ((millis() - start) > 50)
				{
					return;
				}

				if (Serial1.peek() != 'l')
				{
					Serial1.read();
				}
				else if (Serial1.available() >= 5)
				{
					break;
				}
			}

			while (Serial1.available() >= 5)
			{
				Serial1.read();						// Discard 'l'
				auto leftByte = Serial1.read();		// left victim code
				Serial1.read();						// Discard 'r'
				auto rightByte = Serial1.read();	// right victim code
				Serial1.read();						// Discard '\n'

				leftDet++;

				switch (leftByte)
				{
				case 'H':
					leftProb.harmed++;
					break;

				case 'S':
					leftProb.stable++;
					break;

				case 'U':
					leftProb.unharmed++;
					break;

				case 'R':
					leftProb.red++;
					break;

				case 'G':
					leftProb.green++;
					break;

				case 'Y':
					leftProb.yellow++;
					break;

				case 'N':
					leftProb.none++;
					break;

				default:
					leftDet--;
					break;
				}

				rightDet++;

				switch (rightByte)
				{
				case 'H':
					rightProb.harmed++;
					break;

				case 'S':
					rightProb.stable++;
					break;

				case 'U':
					rightProb.unharmed++;
					break;

				case 'R':
					rightProb.red++;
					break;

				case 'G':
					rightProb.green++;
					break;

				case 'Y':
					rightProb.yellow++;
					break;

				case 'N':
					rightProb.none++;
					break;

				default:
					rightDet--;
					break;
				}
			}
		}

		VisVictimProb getVictims(bool left)
		{
			if (left)
			{
				if (leftDet > 0)
				{
					return leftProb / leftDet;
				}
				else
				{
					return VisVictimProb();
				}
			}
			else
			{
				if (rightDet > 0)
				{
					return rightProb / rightDet;
				}
				else
				{
					return VisVictimProb();
				}
			}
		}

		void newField()
		{
			leftDet = 0;
			rightDet = 0;

			leftProb = VisVictimProb();
			rightProb = VisVictimProb();
		}
	}
}