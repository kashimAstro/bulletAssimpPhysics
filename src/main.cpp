#include "ofMain.h"
#include "xApp.h"
#include "ofAppGlutWindow.h"

int main( ){
    ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024,768, OF_WINDOW);
	ofRunApp( new xApp());
}
