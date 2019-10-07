#ifndef MESSAGES_H
#define MESSAGES_H

#include <string>

class Messages {
	private :
		char level;
		/*	0x0=>cmd
			0x1=>dbg
			0x2=>warning
			0x3=>error
		*/
		char complementaryID;
		/*
			When level==0x00
			0x0=> sensor & commands
			0x1=> Unassigned for the moment
			0x2=> Unassigned for the moment
			0x3=> Unassigned for the moment

			When level ==0x03
			0x0 => Commmunication
		*/
		char id;
		/*
			From Pc Application to Raspberry Pi :
			WHEN complementaryID equals 0x0
			0x08 => Front
			0x04 => Back
			0x02 => Left
			0x01 => Right

			From Raspberry Pi to Pc Application :
			When complementaryID equals 0x0 :
			0x00=> Unassigned for the moment
			0x01=> Wheels LR
			0x02=> Speed
			0x03=> US FB
			0x04=> US Left
			0x05=> US Right
			0x06=> Battery
			0x07=> Steering wheel

			When complementaryId equals 0x00 & level==0x03
			0x01=>Can error
		*/

		int32_t value;
		int32_t value2;

	public : 
		Messages(char l, char cid, char main_id,int32_t val, int32_t val2);
		~Messages();

		void display();

		//Getters
		char getLevel(){
			return level;
		}

		char getComplementaryID(){
			return complementaryID;
		}

		char getId(){
			return id;
		}

		int32_t getValue(){
			return value;
		}

		int32_t getValue2(){
			return value2;
		}

		//Static functions, could be in another file
		static Messages decode(std::string receivedBytes);
		static std::string encode(Messages msg);
};
#endif
