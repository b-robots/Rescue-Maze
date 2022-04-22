/*
This private part of the Library is responsible for the communication with the RasPI for the camera recognition.
*/

#include "../header/CamRec.h"
#include "../header/Math.h"

namespace JAFD
{
	namespace CamRec
	{
		namespace
		{
			Victim leftV;
			Victim rightV;
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

			if (!Serial1.available()) {
				return;
			}

			// Wait for at least one full package
			while (true)
			{
				if ((millis() - start) > 10)
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

				switch (leftByte)
				{
				//case 'H':
				//	break;

				//case 'S':
				//	break;

				//case 'U':
				//	break;

				case 'R':
					leftV = Victim::red;
					break;

				case 'G':
					leftV = Victim::green;
					break;

				case 'Y':
					leftV = Victim::yellow;
					break;

				default:
					leftV = Victim::none;
					break;
				}

				switch (rightByte)
				{
					//case 'H':
					//	break;

					//case 'S':
					//	break;

					//case 'U':
					//	break;

				case 'R':
					rightV = Victim::red;
					break;

				case 'G':
					rightV = Victim::green;
					break;

				case 'Y':
					rightV = Victim::yellow;
					break;

				default:
					rightV = Victim::none;
					break;
				}
			}

			while (Serial1.available()) {
				Serial1.read();
			}
		}

		Victim getVictim(bool left)
		{
			size_t maxIdx;

			if (left)
			{
				return leftV;
				//// green, harmed, red, stable, unharmed, yellow
				//int arr[] = {leftConsecutiveCnt.green, leftConsecutiveCnt.harmed, leftConsecutiveCnt.red, leftConsecutiveCnt.stable, leftConsecutiveCnt.unharmed, leftConsecutiveCnt.yellow};
				//maxIdx = argMax(arr, sizeof(arr) / sizeof(arr[0]));
				//consecutiveCnt = arr[maxIdx];
				
			}
			else
			{
				return rightV;
				//// green, harmed, red, stable, unharmed, yellow
				//int arr[] = { rightConsecutiveCnt.green, rightConsecutiveCnt.harmed, rightConsecutiveCnt.red, rightConsecutiveCnt.stable, rightConsecutiveCnt.unharmed, rightConsecutiveCnt.yellow };
				//maxIdx = argMax(arr, sizeof(arr) / sizeof(arr[0]));
				//consecutiveCnt = arr[maxIdx];
			}

			//switch (maxIdx)
			//{
			//case 0:
			//	return Victim::green;
			//case 1:
			//	return Victim::harmed;
			//case 2:
			//	return Victim::red;
			//case 3:
			//	return Victim::stable;
			//case 4:
			//	return Victim::unharmed;
			//case 5:
			//	return Victim::yellow;
			//default:
			//	return Victim::none;
			//	break;
			//}
		}
	}
}