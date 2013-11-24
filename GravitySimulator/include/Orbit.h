#include <math.h>
#include <string>
#include <OgreVector3.h>
#include <OgreSceneManager.h>

#pragma once

typedef Ogre::Vector3 Vector3;

//Strategy 
//- for each object calculate closest object (simplify to two body problem)
//- 

#define PI          3.14159265358979323846
#define RADEG       (180.0/PI)
#define DEGRAD      (PI/180.0)
#define sind(x)     sin((x)*DEGRAD)
#define cosd(x)     cos((x)*DEGRAD)
#define tand(x)     tan((x)*DEGRAD)
#define asind(x)    (RADEG*asin(x))
#define acosd(x)    (RADEG*acos(x))
#define atand(x)    (RADEG*atan(x))
#define atan2d(y,x) (RADEG*atan2((y),(x)))

static double DAYS_PER_YEAR = 365.256898326;
static double DEGREES = 360;
const double GRAVITY = 0.0000001;//6.6742E-11;



enum EccentricityType
{
  CIRCULAR,
  ELLIPTIC,
  PARABOLIC,
  HYPERBOLIC,
  BAD_ECCENTRICITY
};

struct OrbitalCartesian
{
  double x;
  double y;
  double r;
};

struct Orbital
{
  double focusMass;

  double eccentricity;//e
  EccentricityType type;
  double semimajorAxis;//a
  double p;//??
  double trueAnomaly;//nu
  double inclination;//i
  double longitudeOfAscending;//big omega
  double argumentOfPeriapsis;//small omega
  double period;
  double dailyMotion;//mean motion
  long timeAtPerihelion;

  double getTimeSincePerihelion(long epoch)
  {
    return epoch - timeAtPerihelion;
  }

  double getMeanAnomaly(long epoch)
  {
    double STEPS = 600;//steps per second
    return dailyMotion * (getTimeSincePerihelion(epoch) / STEPS);

  }

  OrbitalCartesian getCartesian(double angle)
  {
    OrbitalCartesian cartesian;
    cartesian.x = -semimajorAxis * eccentricity + semimajorAxis * cos(angle);
    cartesian.y = semimajorAxis * sqrt(1 - pow(eccentricity, 2)) * sin(angle);
    cartesian.r = semimajorAxis * (1 - eccentricity * cos(angle));
    return cartesian;
  }


  void draw(Ogre::SceneManager* sceneManager)
  {

    if(sceneManager->hasManualObject("Ellipse")) sceneManager->destroyManualObject("Ellipse");
    if(sceneManager->hasSceneNode("EllipseNode")) sceneManager->destroySceneNode("EllipseNode");
    if(type == CIRCULAR || type == ELLIPTIC)
    {
      Ogre::ManualObject* Ellipse = sceneManager->createManualObject("Ellipse");
      Ogre::SceneNode* EllipseNode = sceneManager->getRootSceneNode()->createChildSceneNode("EllipseNode");
      Ellipse->begin("BaseWhite", Ogre::RenderOperation::OT_LINE_STRIP);
      const float accuracy = 30;
      //const float radiusX = earthOrbit.semimajorAxis;
      //const float radiusY = earthOrbit.semimajorAxis * sqrt(1 - pow(earthOrbit.eccentricity, 2));
      unsigned int index = 0;
      for(float theta = 0; theta <= 2 * PI; theta += PI / accuracy)
      {
        OrbitalCartesian cartesian = getCartesian(theta);
        Vector3 position(cartesian.x, 0, cartesian.y);
        //Vector3 position(cos(theta)*radiusX, 0, sin(theta)*radiusY);
        Ellipse->position(position);
        Ellipse->index(index++);
      }
      Ellipse->end();
      EllipseNode->attachObject(Ellipse);

      Vector3 rotationAxis = Ogre::Quaternion(Ogre::Radian(longitudeOfAscending), Vector3::UNIT_Y) * Vector3::UNIT_X;
      Ogre::Radian rotationAngle(argumentOfPeriapsis + trueAnomaly); 
      Ogre::Quaternion rotation(rotationAngle, rotationAxis);
      EllipseNode->rotate(rotation);
    }
  }
};





double rev( double x )
{
  return  x - floor(x/360.0)*360.0;
}


double cubedRoot( double x )
{
  if ( x > 0.0 )
    return exp( log(x) / 3.0 );
  else if ( x < 0.0 )
    return -cubedRoot(-x);
  else /* x == 0.0 */
    return 0.0;
}

long doubleToLong(double value)
{
  return floor(value + 0.5);

}

double standardGravitationalParameter(double gravitationalConstant, double mass)
{
  return gravitationalConstant * mass;
}

double eccentricity(double orbitalEnergy, double angularMomentum, double reducedMass, double standardGravitationalParameter)
{
  double specificOrbitalEnergy = orbitalEnergy / reducedMass;
  double specificRelativeAngularMomentum = angularMomentum / reducedMass;
  return sqrt(1 + ((2 * specificOrbitalEnergy * pow(specificRelativeAngularMomentum, 2))/(pow(standardGravitationalParameter, 2))));
}

double eccentricity(double radiusAtPeriapsis, double radiusAtApoapsis)
{
  return (radiusAtApoapsis - radiusAtPeriapsis)/(radiusAtApoapsis + radiusAtPeriapsis);
}

Vector3 eccentricityVector(Vector3 position, Vector3 velocity, double standardGravitationalParameter)
{
  return ((pow(velocity.length(), 2) - standardGravitationalParameter / position.length()) * position - (position.dotProduct(velocity)) * velocity) / standardGravitationalParameter;
}

std::string eccentricityToString(double e)
{
  if(e == 0) return "circular";
  if(e > 0 && e < 1) return "elliptic";
  if(e == 1) return "parabolic";
  if(e > 1) return "hyperbolic";
  return "null";
}

EccentricityType eccentricityToType(double e)
{
  if(e == 0) return CIRCULAR;
  if(e > 0 && e < 1) return ELLIPTIC;
  if(e == 1) return PARABOLIC;
  if(e > 1) return HYPERBOLIC;
  return BAD_ECCENTRICITY;
}

Vector3 angularMomentum(Vector3 position, Vector3 velocity)
{
  return position.crossProduct(velocity);
}

double mechanicalEnergy(double positionMagnitude, double velocityMagnitude, double standardGravitationalParameter)
{
  return (pow(velocityMagnitude, 2) / 2) - (standardGravitationalParameter / positionMagnitude);
}

//mass centre at 0,0,0
Orbital calculateOrbital(Vector3 position, Vector3 velocity, double focusMass, long epoch)
{

  Orbital orbital;
  orbital.focusMass = focusMass;

  double sgp = standardGravitationalParameter(0.0000001, focusMass);
  Vector3 am = angularMomentum(position, velocity);
  Vector3 nodeVec = Vector3(0,0,1).crossProduct(am);
  Vector3 eVec = eccentricityVector(position, velocity, sgp);

  orbital.eccentricity = eVec.length();
  
  double me = mechanicalEnergy(position.length(), velocity.length(), sgp);

  if(orbital.eccentricity != 1)
  {
    
    orbital.semimajorAxis = -(sgp / (2 * me));
    orbital.p = orbital.semimajorAxis * (1 - pow(orbital.eccentricity, 2));
  }
  else
  {
    orbital.semimajorAxis = -100000;
    orbital.p = pow(am.length(), 2) * sgp;
  }

  orbital.argumentOfPeriapsis = acos(nodeVec.dotProduct(eVec) / (nodeVec.length() * orbital.eccentricity));  
  orbital.inclination = acos(am.z / am.length());
  orbital.longitudeOfAscending = acos(nodeVec.x / nodeVec.length());
  orbital.trueAnomaly = acos(eVec.dotProduct(position) / (orbital.eccentricity * position.length()));

  orbital.period = 2 * PI * sqrt(pow(orbital.semimajorAxis, 3) / sgp);
  orbital.dailyMotion = DEGREES / orbital.period;

  //TODO: calculate time
  double timeFromPerihelion = orbital.period / orbital.trueAnomaly;
  orbital.timeAtPerihelion = epoch - doubleToLong(timeFromPerihelion);

  orbital.type =  eccentricityToType(orbital.eccentricity);

  return orbital;
}






class Orbit
{
  //Anomaly anomaly;

  double mass;

  Vector3 periapsis;
  Vector3 apoapsis;
  Vector3 focus;

  double inclination;
  double capitalOmega;//longitude of ascending node
  double smallOmega;//angle from the ascending node to the perihelion

  double semimajor;
  double eccentricity;
  int periapsisEpoch;

  double getPeriapsisDistance()
  {
    return semimajor * (1 - eccentricity);
  }

  double getApoapsisDistance()
  {
    return semimajor * (1 + eccentricity);
  }

  double getOrbitalPeriod()
  {
    return DAYS_PER_YEAR * pow(semimajor, 1.5) / sqrt(1 + mass); 
  }

  double getDegreesPerDay()
  {
    return DEGREES / getOrbitalPeriod();
  }

  int getTimeSincePeriapsis(int currentEpoch)
  {
    return currentEpoch - periapsisEpoch;
  }

  double getMeanAnomaly(int currentEpoch)
  {
    return getDegreesPerDay() * getTimeSincePeriapsis(currentEpoch);
  }


};