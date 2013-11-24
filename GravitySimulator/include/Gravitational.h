

#include <OgreVector3.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include "Orbit.h"


class Gravitational
{
public:

  Vector3 position;
  Vector3 velocity;
  Vector3 acceleration;
  
  double mass;
  double radius;

  Ogre::SceneNode* node;
  Ogre::Entity* entity;

  std::string name;

  Gravitational(std::string name, Ogre::SceneManager* sceneManager, Vector3 position, Vector3 velocity, Vector3 acceleration, double mass, double radius)
    : position(position),
      velocity(velocity),
      acceleration(acceleration),
      mass(mass),
      radius(radius),
      name(name)
  {
    //sceneManager->setDisplaySceneNodes(true);
    node = sceneManager->getRootSceneNode()->createChildSceneNode();
    entity = sceneManager->createEntity(name, "ogrehead.mesh");//Ogre::SceneManager::PT_SPHERE);
    node->attachObject(entity);
    node->setPosition(position);
  }

  void applyForce(double force, Vector3 unitDirection, const double& timeDelta)
  {
    velocity += force * unitDirection * (1 / mass) * timeDelta;
  }

  void applyImpulse(Vector3 impulse)
  {
    acceleration += impulse;
  }

  void applyGravitation(Gravitational* gravitational, const double& timeDelta)
  {
    double distance = position.distance(gravitational->position);
    
    Vector3 unitDirection = position - gravitational->position;
    unitDirection.normalise();

    applyForce(-GRAVITY * mass * gravitational->mass / (distance * distance), unitDirection, timeDelta);

    //std::cout << name << " force: " << force << std::endl;
  }

  void update(const double& timeDelta)
  {
    velocity += acceleration * timeDelta;
    position += velocity * timeDelta;

    node->setPosition(position);
  }

  static bool isColliding(Gravitational g1, Gravitational g2)
  {
    return g1.position.distance(g2.position) < g1.radius + g2.radius;
  }

  static Vector3 getCircularOrbitVelocity(Gravitational* parent, Vector3 position, double mass)
  {
    Vector3 distance = parent->position - position;
    double radius = distance.length();
    double velocity = sqrt(GRAVITY * (parent->mass + mass) / radius);

    return distance.perpendicular().normalisedCopy() * velocity;
  }

  Orbital orbital;

  void setOrbital(Ogre::SceneManager* sceneManager, Orbital orbital)
  {
    this->orbital = orbital;

    orbital.draw(sceneManager);

    Vector3 rotationAxis = Ogre::Quaternion(Ogre::Radian(orbital.longitudeOfAscending), Vector3::UNIT_Y) * Vector3::UNIT_X;
    Ogre::Radian rotationAngle(orbital.argumentOfPeriapsis + orbital.trueAnomaly); 
    Ogre::Quaternion rotation(rotationAngle, rotationAxis);
    node->setOrientation(rotation);
  }

  void setPosition(long epoch)
  {
    OrbitalCartesian cartesian = orbital.getCartesian(orbital.getMeanAnomaly(epoch));
    position = Vector3(cartesian.x, 0, cartesian.y);
    node->setPosition(position);
  }

  void accelerate(Ogre::SceneManager* sceneManager, double multiplier, double focusMass, long epoch)
  {
    velocity *= 1.001;
    setOrbital(sceneManager, calculateOrbital(position, velocity, focusMass, epoch));
  }
};

