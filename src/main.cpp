///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*********************************************************************************
 ** This sample demonstrates how to capture 3D point cloud and detected objects **
 **      with the ZED SDK and display the result in an OpenGL window. 	        **
 *********************************************************************************/

// Standard includes
#include <iostream>
#include <fstream>
#include "CameraOperations/CameraZED.hpp"
#include <string>



int main(int argc, char **argv) 
{
	int cameraSerialNumber = 0;
    bool useSVO = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--serialNumber" && i + 1 < argc) {
            cameraSerialNumber = std::stoi(argv[++i]); // convert to int
        }
        else if (arg == "--useSVO") {
            useSVO = true;
        }
    }
	CameraOperations::CameraZED* camera = new CameraOperations::CameraZED(cameraSerialNumber, useSVO);
}

