#pragma once
#include <string>
#include <thread>

namespace CameraOperations {
	class CameraZED
	{
	public:
		CameraZED(int cameraSerialNumber, bool useSVO);
		~CameraZED();

	private:		
		int cameraSerialNumber;
		bool useSVO;
		//std::thread* cameraThread;

	private:
		void cameraTaskFunction();
	};
};


