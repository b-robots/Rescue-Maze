#include "../header/CamRec.h"
#include "../header/Math.h"

namespace SIAL
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
				case 'H':
					leftV = Victim::harmed;
					break;
				case 'S':
					leftV = Victim::stable;
					break;
				case 'U':
					leftV = Victim::unharmed;
					break;
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
				case 'H':
					rightV = Victim::harmed;
					break;
				case 'S':
					rightV = Victim::stable;
					break;
				case 'U':
					rightV = Victim::unharmed;
					break;
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

			}
			else
			{
				return rightV;
			}
		}
	}
}