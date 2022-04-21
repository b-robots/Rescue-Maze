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
			VisVictimCount leftConsecutiveCnt;
			VisVictimCount rightConsecutiveCnt;
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
				case 'H':
					leftConsecutiveCnt.harmed++;
					leftConsecutiveCnt.green = 0;
					leftConsecutiveCnt.red = 0;
					leftConsecutiveCnt.stable = 0;
					leftConsecutiveCnt.unharmed = 0;
					leftConsecutiveCnt.yellow = 0;
					break;

				case 'S':
					leftConsecutiveCnt.harmed = 0;
					leftConsecutiveCnt.green = 0;
					leftConsecutiveCnt.red = 0;
					leftConsecutiveCnt.stable++;
					leftConsecutiveCnt.unharmed = 0;
					leftConsecutiveCnt.yellow = 0;
					break;

				case 'U':
					leftConsecutiveCnt.harmed = 0;
					leftConsecutiveCnt.green = 0;
					leftConsecutiveCnt.red = 0;
					leftConsecutiveCnt.stable = 0;
					leftConsecutiveCnt.unharmed++;
					leftConsecutiveCnt.yellow = 0;
					break;

				case 'R':
					leftConsecutiveCnt.harmed = 0;
					leftConsecutiveCnt.green = 0;
					leftConsecutiveCnt.red++;
					leftConsecutiveCnt.stable = 0;
					leftConsecutiveCnt.unharmed = 0;
					leftConsecutiveCnt.yellow = 0;
					break;

				case 'G':
					leftConsecutiveCnt.harmed = 0;
					leftConsecutiveCnt.green++;
					leftConsecutiveCnt.red = 0;
					leftConsecutiveCnt.stable = 0;
					leftConsecutiveCnt.unharmed = 0;
					leftConsecutiveCnt.yellow = 0;
					break;

				case 'Y':
					leftConsecutiveCnt.harmed = 0;
					leftConsecutiveCnt.green = 0;
					leftConsecutiveCnt.red = 0;
					leftConsecutiveCnt.stable = 0;
					leftConsecutiveCnt.unharmed = 0;
					leftConsecutiveCnt.yellow++;
					break;

				default:
					leftConsecutiveCnt.harmed = 0;
					leftConsecutiveCnt.green = 0;
					leftConsecutiveCnt.red = 0;
					leftConsecutiveCnt.stable = 0;
					leftConsecutiveCnt.unharmed = 0;
					leftConsecutiveCnt.yellow = 0;
					break;
				}

				switch (rightByte)
				{
				case 'H':
					rightConsecutiveCnt.harmed++;
					rightConsecutiveCnt.green = 0;
					rightConsecutiveCnt.red = 0;
					rightConsecutiveCnt.stable = 0;
					rightConsecutiveCnt.unharmed = 0;
					rightConsecutiveCnt.yellow = 0;
					break;

				case 'S':
					rightConsecutiveCnt.harmed = 0;
					rightConsecutiveCnt.green = 0;
					rightConsecutiveCnt.red = 0;
					rightConsecutiveCnt.stable++;
					rightConsecutiveCnt.unharmed = 0;
					rightConsecutiveCnt.yellow = 0;
					break;

				case 'U':
					rightConsecutiveCnt.harmed = 0;
					rightConsecutiveCnt.green = 0;
					rightConsecutiveCnt.red = 0;
					rightConsecutiveCnt.stable = 0;
					rightConsecutiveCnt.unharmed++;
					rightConsecutiveCnt.yellow = 0;
					break;

				case 'R':
					rightConsecutiveCnt.harmed = 0;
					rightConsecutiveCnt.green = 0;
					rightConsecutiveCnt.red++;
					rightConsecutiveCnt.stable = 0;
					rightConsecutiveCnt.unharmed = 0;
					rightConsecutiveCnt.yellow = 0;
					break;

				case 'G':
					rightConsecutiveCnt.harmed = 0;
					rightConsecutiveCnt.green++;
					rightConsecutiveCnt.red = 0;
					rightConsecutiveCnt.stable = 0;
					rightConsecutiveCnt.unharmed = 0;
					rightConsecutiveCnt.yellow = 0;
					break;

				case 'Y':
					rightConsecutiveCnt.harmed = 0;
					rightConsecutiveCnt.green = 0;
					rightConsecutiveCnt.red = 0;
					rightConsecutiveCnt.stable = 0;
					rightConsecutiveCnt.unharmed = 0;
					rightConsecutiveCnt.yellow++;
					break;

				default:
					rightConsecutiveCnt.harmed = 0;
					rightConsecutiveCnt.green = 0;
					rightConsecutiveCnt.red = 0;
					rightConsecutiveCnt.stable = 0;
					rightConsecutiveCnt.unharmed = 0;
					rightConsecutiveCnt.yellow = 0;
					break;
				}
			}

			while (Serial1.available()) {
				Serial1.read();
			}
		}

		Victim getVictim(bool left, uint16_t& consecutiveCnt)
		{
			size_t maxIdx;

			if (left)
			{
				// green, harmed, red, stable, unharmed, yellow
				int arr[] = {leftConsecutiveCnt.green, leftConsecutiveCnt.harmed, leftConsecutiveCnt.red, leftConsecutiveCnt.stable, leftConsecutiveCnt.unharmed, leftConsecutiveCnt.yellow};
				maxIdx = argMax(arr, sizeof(arr) / sizeof(arr[0]));
				consecutiveCnt = arr[maxIdx];
				
			}
			else
			{
				// green, harmed, red, stable, unharmed, yellow
				int arr[] = { rightConsecutiveCnt.green, rightConsecutiveCnt.harmed, rightConsecutiveCnt.red, rightConsecutiveCnt.stable, rightConsecutiveCnt.unharmed, rightConsecutiveCnt.yellow };
				maxIdx = argMax(arr, sizeof(arr) / sizeof(arr[0]));
				consecutiveCnt = arr[maxIdx];
			}

			switch (maxIdx)
			{
			case 0:
				return Victim::green;
			case 1:
				return Victim::harmed;
			case 2:
				return Victim::red;
			case 3:
				return Victim::stable;
			case 4:
				return Victim::unharmed;
			case 5:
				return Victim::yellow;
			default:
				return Victim::none;
				break;
			}
		}
	}
}