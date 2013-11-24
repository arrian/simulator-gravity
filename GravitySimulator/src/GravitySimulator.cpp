#include "GravitySimulator.h"

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#include "../res/resource.h"
#endif


#include "Orbit.h"
#include <OgreQuaternion.h>

//-------------------------------------------------------------------------------------
GravitySimulator::GravitySimulator(void)
    : mRoot(0),
    mCamera(0),
    mSceneMgr(0),
    mWindow(0),
    mResourcesCfg(Ogre::StringUtil::BLANK),
    mPluginsCfg(Ogre::StringUtil::BLANK),
    mCameraMan(0),
    mCursorWasVisible(false),
    mShutDown(false),
    mInputManager(0),
    mMouse(0),
    mKeyboard(0),
    planets()
{
}

//-------------------------------------------------------------------------------------
GravitySimulator::~GravitySimulator(void)
{
    if (mCameraMan) delete mCameraMan;

    //Remove ourself as a Window listener
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
    delete mRoot;
}

//-------------------------------------------------------------------------------------
bool GravitySimulator::configure(void)
{
    // Show the configuration dialog and initialise the system
    // You can skip this and use root.restoreConfig() to load configuration
    // settings if you were sure there are valid ones saved in ogre.cfg
    if(mRoot->showConfigDialog())
    {
        // If returned true, user clicked OK so initialise
        // Here we choose to let the system create a default rendering window by passing 'true'
        mWindow = mRoot->initialise(true, "Gravity Simulator");

        // Let's add a nice window icon
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        HWND hwnd;
        mWindow->getCustomAttribute("WINDOW", (void*)&hwnd);
        LONG iconID   = (LONG)LoadIcon( GetModuleHandle(0), MAKEINTRESOURCE(IDI_APPICON) );
        SetClassLong( hwnd, GCL_HICON, iconID );
#endif
        return true;
    }
    else
    {
        return false;
    }
}
//-------------------------------------------------------------------------------------
void GravitySimulator::chooseSceneManager(void)
{
    // Get the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);
}
//-------------------------------------------------------------------------------------
void GravitySimulator::createCamera(void)
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");

    // Position it at 500 in Z direction
    mCamera->setPosition(Ogre::Vector3(0,0,80));
    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0,0,-300));
    mCamera->setNearClipDistance(5);

    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}
//-------------------------------------------------------------------------------------
void GravitySimulator::createFrameListener(void)
{
    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;

    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

    mInputManager = OIS::InputManager::createInputSystem( pl );

    mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, true ));
    mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, true ));

    mMouse->setEventCallback(this);
    mKeyboard->setEventCallback(this);

    //Set initial mouse clipping size
    windowResized(mWindow);

    //Register as a Window listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    mRoot->addFrameListener(this);
}
//-------------------------------------------------------------------------------------
void GravitySimulator::destroyScene(void)
{
}
//-------------------------------------------------------------------------------------
void GravitySimulator::createViewports(void)
{
    // Create one viewport, entire window
    Ogre::Viewport* vp = mWindow->addViewport(mCamera);
    vp->setBackgroundColour(Ogre::ColourValue(0,0,0));

    // Alter the camera aspect ratio to match the viewport
    mCamera->setAspectRatio(
        Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
}
//-------------------------------------------------------------------------------------
void GravitySimulator::setupResources(void)
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }
}
//-------------------------------------------------------------------------------------
void GravitySimulator::createResourceListener(void)
{

}
//-------------------------------------------------------------------------------------
void GravitySimulator::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}
//-------------------------------------------------------------------------------------
void GravitySimulator::go(void)
{
#ifdef _DEBUG
    mResourcesCfg = "resources_d.cfg";
    mPluginsCfg = "plugins_d.cfg";
#else
    mResourcesCfg = "resources.cfg";
    mPluginsCfg = "plugins.cfg";
#endif

    if (!setup())
        return;

    mRoot->startRendering();

    // clean up
    destroyScene();
}
//-------------------------------------------------------------------------------------
bool GravitySimulator::setup(void)
{
    mRoot = new Ogre::Root(mPluginsCfg);

    setupResources();

    bool carryOn = configure();
    if (!carryOn) return false;

    chooseSceneManager();
    createCamera();
    createViewports();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Create any resource listeners (for loading screens)
    createResourceListener();
    // Load resources
    loadResources();

	// Create the scene
    createScene();

	createFrameListener();

    return true;
};

Orbital earthOrbit;
long epoch = 0;

boost::shared_ptr<Gravitational> earth;

//-------------------------------------------------------------------------------------
void GravitySimulator::createScene(void)
{
	// Set ambient light
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

	// Create a light
	Ogre::Light* l = mSceneMgr->createLight("MainLight");
	l->setPosition(20,80,50);


  //Simulation
  boost::shared_ptr<Gravitational> sun(new Gravitational("Sun", mSceneMgr, Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0), 100000000, 10));
  planets.push_back(sun);
  earth = boost::shared_ptr<Gravitational>(new Gravitational("Earth", mSceneMgr, Vector3(100,0,0), Gravitational::getCircularOrbitVelocity(sun.get(), Vector3(0,100,0), 1000000) * Vector3(0,0,1.3), Vector3(0,0,0), 1, 10));//mass 1000000
  planets.push_back(earth);
  //planets.push_back(boost::shared_ptr<Gravitational>(new Gravitational("Mars", mSceneMgr, Vector3(0,0,20), Vector3(0,0,-2), Vector3(0,0,0), 1000000, 10)));

  earthOrbit = calculateOrbital(earth->position, earth->velocity, sun->mass, 0);
  earth->setOrbital(mSceneMgr, earthOrbit);
  /*
  if(earthOrbit.type == CIRCULAR || earthOrbit.type == ELLIPTIC)
  {
    Ogre::ManualObject* Ellipse=mSceneMgr->createManualObject("Ellipse");
    Ogre::SceneNode* EllipseNode=mSceneMgr->getRootSceneNode()->createChildSceneNode("EllipseNode");
    Ellipse->begin("BaseWhite", Ogre::RenderOperation::OT_LINE_STRIP);
    const float accuracy = 30;
    //const float radiusX = earthOrbit.semimajorAxis;
    //const float radiusY = earthOrbit.semimajorAxis * sqrt(1 - pow(earthOrbit.eccentricity, 2));
    unsigned int index = 0;
    for(float theta = 0; theta <= 2 * PI; theta += PI / accuracy)
    {
      OrbitalCartesian cartesian = earthOrbit.getCartesian(theta);
      Vector3 position(cartesian.x, 0, cartesian.y);
      //Vector3 position(cos(theta)*radiusX, 0, sin(theta)*radiusY);
      Ellipse->position(position);
      Ellipse->index(index++);
    }
    Ellipse->end();
    EllipseNode->attachObject(Ellipse);

    Vector3 rotationAxis = Ogre::Quaternion(Ogre::Radian(earthOrbit.longitudeOfAscending), Vector3::UNIT_Y) * Vector3::UNIT_X;
    Ogre::Radian rotationAngle(earthOrbit.argumentOfPeriapsis + earthOrbit.trueAnomaly); 
    Ogre::Quaternion rotation(rotationAngle, rotationAxis);
    EllipseNode->rotate(rotation);
  }*/
}


bool oneDown = false;

//-------------------------------------------------------------------------------------
bool GravitySimulator::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    //Need to capture/update each device
    mKeyboard->capture();
    mMouse->capture();

    mCameraMan->frameRenderingQueued(evt);   // if dialog isn't up, then update the camera



    double elapsedSeconds = evt.timeSinceLastFrame;

    double speedMultiplier = 100;

    epoch += 1;

    if(oneDown) earth->accelerate(mSceneMgr, 0.1, 100000000, epoch);

    earth->setPosition(epoch);

    //Simulate
    /*
    for(std::vector<boost::shared_ptr<Gravitational> >::iterator iter = planets.begin(); iter != planets.end(); ++iter)
    {
      (*iter)->update(evt.timeSinceLastFrame);

      //Simulate
      for(std::vector<boost::shared_ptr<Gravitational> >::iterator iter2 = planets.begin(); iter2 != planets.end(); ++iter2)
      {
        if((*iter) != (*iter2))
        {
          //(*iter)->applyGravitation((*iter2).get(), evt.timeSinceLastFrame * speedMultiplier);
        }

        (*iter)->setPosition(epoch);
        //(*iter)->update(evt.timeSinceLastFrame * speedMultiplier);
      }
    }
    */

    return true;
}


//-------------------------------------------------------------------------------------
bool GravitySimulator::keyPressed( const OIS::KeyEvent &arg )
{
    
    if(arg.key == OIS::KC_F5)   // refresh all textures
    {
        Ogre::TextureManager::getSingleton().reloadAll();
    }
    else if (arg.key == OIS::KC_SYSRQ)   // take a screenshot
    {
        mWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
    }
    else if (arg.key == OIS::KC_ESCAPE)
    {
        mShutDown = true;
    }
    else if(arg.key == OIS::KC_1)
    {
      oneDown = true;
    }

    mCameraMan->injectKeyDown(arg);
    return true;
}

bool GravitySimulator::keyReleased( const OIS::KeyEvent &arg )
{
      if(arg.key == OIS::KC_1)
      {
        oneDown = false;
    }

    mCameraMan->injectKeyUp(arg);
    return true;
}

bool GravitySimulator::mouseMoved( const OIS::MouseEvent &arg )
{
    mCameraMan->injectMouseMove(arg);
    return true;
}

bool GravitySimulator::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    mCameraMan->injectMouseDown(arg, id);
    return true;
}

bool GravitySimulator::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
{
    mCameraMan->injectMouseUp(arg, id);
    return true;
}

//Adjust mouse clipping area
void GravitySimulator::windowResized(Ogre::RenderWindow* rw)
{
    unsigned int width, height, depth;
    int left, top;
    rw->getMetrics(width, height, depth, left, top);

    const OIS::MouseState &ms = mMouse->getMouseState();
    ms.width = width;
    ms.height = height;
}

//Unattach OIS before window shutdown (very important under Linux)
void GravitySimulator::windowClosed(Ogre::RenderWindow* rw)
{
    //Only close for window that created OIS (the main window in these demos)
    if( rw == mWindow )
    {
        if( mInputManager )
        {
            mInputManager->destroyInputObject( mMouse );
            mInputManager->destroyInputObject( mKeyboard );

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }
}


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
      //Console
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#ifdef _DEBUG
      AllocConsole();
      freopen("CONOUT$","wb",stdout);
#endif
#endif

        // Create application object
        GravitySimulator app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
