#define _USE_MATH_DEFINES
#define SDL_MAIN_HANDLED

#include <iostream>
#include <algorithm>
#include <fstream>   
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <SDL.h>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>

namespace NC
{
	struct LightProperties
	{
		glm::vec3 ambient;
		glm::vec3 diffuse;
		glm::vec3 specular;
		float specularPower;
		float totalReflection;
	};

	struct Vertex
	{
		glm::vec3 position;
		glm::vec3 normal;
		glm::vec2 texCoord;
	};

	struct Ray
	{
		glm::vec3 position;
		glm::vec3 direction;

		Ray(glm::vec3 pos = glm::vec3(0.0f), glm::vec3 dir = glm::vec3(1.0f)) : position(pos), direction(dir) {}
	};

	class Entity
	{
	public:

		glm::vec3 position;
		glm::vec3 rotation;
		glm::vec3 scale;

		//Set the position of the entity
		bool setPos(glm::vec3 newPos);

		//Set the rotation of the entity
		bool setRot(glm::vec3 newRot);

		//Set the scale of the entity
		bool setSca(glm::vec3 newSca);
	};

	class PointLight : public Entity
	{
	public:
		LightProperties properties;
		glm::vec3 lightIntensity; 

		PointLight(glm::vec3 _intensity, LightProperties _properties) : lightIntensity(_intensity), properties(_properties) {}
	};

	class AreaLight : public Entity
	{
	public:
		LightProperties properties;
		glm::vec3 lightIntensity;
		glm::vec3 relativeStartPos;
		glm::vec3 relativeEndPos;

		AreaLight(glm::vec3 _intensity, LightProperties _properties, glm::vec3 _relativeStartPos, glm::vec3 _relativeEndPos) : lightIntensity(_intensity), properties(_properties), relativeStartPos(_relativeStartPos), relativeEndPos(_relativeEndPos) {}

		std::vector<glm::vec3> SampleLocations(int sampleAmountX, int sampleAmountY, int sampleAmountZ);

		std::vector<glm::vec3> SampleLocationsWithNoise(int sampleAmountX, int sampleAmountY, int sampleAmountZ);
	};

	class Intersectable : public Entity
	{
	public:

		enum INTERSECTION_TYPE
		{
			DEFAULT,
			RAY_TO_SPHERE,
			RAY_TO_PLANE,
			RAY_TO_TRIANGLE,
			RAY_TO_AABB,
			RAY_TO_OBJECT
		};

		INTERSECTION_TYPE intersectionType;
		LightProperties colours;

	};

	class AABB : public Intersectable
	{
	public:
		glm::vec3 minPoints;
		glm::vec3 maxPoints;

		AABB();
	};

	class Sphere : public Intersectable
	{
	public:
		AABB boundingBox;
		float radius;

		Sphere();
	};

	class Plane : public Intersectable
	{
	public:
		AABB boundingBox;
		glm::vec3 normalToPlane;

		Plane();
	};

	class Triangle : public Intersectable
	{
	public:
		AABB boundingBox;
		std::vector<Vertex> vertices;

		Triangle();
		Triangle(std::vector<Vertex> _vertices);
	};

	class Model : public Intersectable
	{
	public:
		AABB boundingBox;
		std::vector<Triangle> triangles;

		Model();
	};

	class Camera : public Entity
	{
	public:
		glm::mat4 projMat;
	};

	struct OctTreeNode
	{
	public:
		glm::vec3 position, extents;

		int maxDepth, currentDepth, splitAmount;

		std::set<Intersectable*> containedIntersectables;

		bool endNode = true;

		std::vector<OctTreeNode*> nodes = {};

		OctTreeNode(glm::vec3 _position, glm::vec3 _extents, int _maxDepth = 8, int _splitAmount = 2, int _currentDepth = 0);

		~OctTreeNode();

		bool FillOctTree(std::vector<Intersectable*> intersectables);

		std::set<Intersectable*> GetAllPotentialIntersectables(Ray ray);

	};

	struct IntersectionTests
	{
		//Result data from an intersection test
		struct Result
		{
			bool hit = false;
			float length;
			glm::vec3 normal;
			glm::vec3 intersectionPoint;
		};

		//Determine correct Intersection test based on INTERSECTION_TYPE provided and returns a call of relevant intersection test
		static Result rayEntity(Ray ray, Intersectable* entity, Intersectable::INTERSECTION_TYPE type);

		//Intersection test between a ray and sphere
		static Result raySphere(Ray ray, Sphere sphere);

		//Intersection test between a ray and a plane
		static Result rayPlane(Ray ray, Plane plane);

		//Intersection test between a ray and a triangle
		static Result rayTriangle(Ray ray, Triangle triangle);

		//Interesction test between a ray and an AABB
		static Result rayAABB(Ray ray, AABB aabb);

		//Intersection test between a ray and an object (consisting of triangles)
		static Result rayModel(Ray ray, Model object);

		//Intersection test between a ray and an oct-tree node
		static bool rayOctTreeNode(Ray ray, OctTreeNode* octTree);

		//Intersection test between a ray and an AABB (bounding box)
		static bool OctTreeBoxToAABB(OctTreeNode octTree, AABB aabb);

		//Collision data
		struct CollisionResult
		{
			Result result;
			Intersectable* intersectedObject;
		};

		//Identifies from the list of intersectables which intersect with the ray and returns the closest one.
		//WARNING: If none of them intersect then the intersectedObject value will be nullptr, use result.hit to identify if any have been hit first
		static CollisionResult ClosestCollisionWithRay(Ray ray, std::vector<Intersectable*> intersectables, std::vector<int> indexIgnores = {});
	};


	class GraphicsRenderer
	{
	public:


		bool fillPrimitives = true;

		float width;

		float height;

		float fieldOfView = glm::tan(glm::radians(45.0f) / 2.0f);

		glm::vec3 * *screenBuffer;

		std::mutex screenBufferLock;

		glm::vec3 * *processBuffer;

		std::mutex processBufferLock;

		glm::vec3 * *eraseBuffer;

		glm::vec3 ambientLightIntensity = glm::vec3(0.1f);

		SDL_Window* window = NULL;

		SDL_Surface* surface = NULL;

		Camera camera;

		std::vector<PointLight> pointLights = {};

		std::vector<AreaLight> areaLights = {};

		std::mutex areaLightsLock;

		glm::vec3 sampleAmount = glm::vec3(6, 6, 1);

		std::vector<Intersectable*> intersectablePrimitives = {};

		std::vector<Intersectable*> intersectableEntities = {};

		bool useOctTree = false;

		OctTreeNode octTree = OctTreeNode(glm::vec3(0.0f), glm::vec3(30.f));

		std::mutex pollEventsLock;

		bool pollEvents = true;

		std::mutex keyMapLock;

		std::map<char, bool> keyMap;

		bool multiThreading = true;

		enum SHADING_TYPE
		{
			NO_PHONG_SHADING,
			NO_SHADOWS,
			HARD_SHADOWS,
			SOFT_SHADOWS,
			RECURSIVE_REFLECTION
		};

		SHADING_TYPE shadingType = NO_SHADOWS;

		bool renderComplexGeometry = false;

		bool renderPrimitiveGeometry = true;

		bool HDR = false;

		int recursiveReflectionMaxDepth = 3;

		GraphicsRenderer(float _width, float _height);

		~GraphicsRenderer();

		//Take a surface and set the colour of a specified pixel
		void SetSDLSurfacePixel(SDL_Surface* surface, int x, int y, Uint32 colour);

		//Converts a vec3 to an RGBA32 formatted Uint32
		Uint32 Vec3ToUint32(glm::vec3 value);

		//For a given pixel, this function will determine what colour it is by processing all the models and primitives via a ray
		glm::vec3 DrawPixel(glm::vec2 pixelPos, Camera camera);

		//Returns a thread object that runs the method DrawPixelThread
		std::thread RtnDrawPixelThread(glm::vec2 pixelPos, Camera camera);

		//Returns a thread object that runs the method DrawHorizontalPixelsThread
		std::thread RtnDrawHorizontalPixelsThread(int pixelY, int amountOfLines, int width, Camera camera);

		//Calls DrawPixel and stores the value in processBuffer
		void DrawPixelThread(glm::vec2 pixelPos, Camera camera);

		//Calls DrawPixel across a specified amount of horizontal lines of a certain width starting from pixelY to pixelY + amountofLines and stores the value for each pixel in processBuffer
		void DrawHorizontalPixelsThread(int pixelY, int amountOfLines, int width, Camera camera);

		//Creates a sphere primitive given the data and records it
		bool CreateSphere(float radius, LightProperties colour, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale);

		//Creates a plane primitive given the data and records it
		bool CreatePlane(glm::vec3 normal, LightProperties colour, glm::vec3 position);

		//Creates a triangle primitive given the data and records it
		bool CreateTriangle(Vertex vertexA, Vertex vertexB, Vertex vertexC, LightProperties colour);

		//Creates an object (consisiting of triangles) given the data and records it
		bool CreateObject(std::string filePath, LightProperties colour, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale);

		//Creates a point light in the specified location with the intensity
		bool CreatePointLight(glm::vec3 intensity, glm::vec3 position);

		//Creates an area light in the specified location with the specified intensity and its start and end position
		bool CreateAreaLight(glm::vec3 intensity, glm::vec3 position, glm::vec3 startPos, glm::vec3 endPos); 

		//Takes a given ray and performs reflections up to maxDepth and returns reflected colour
		glm::vec3 TraceReflectiveRay(Ray ray, glm::vec3 specularCoefficient, int maxDepth, int depth);

		//Call this function to display the image from screenBuffer to a window
		void DisplayBuffer();

		//Call this function to render the image a single time to the processBuffer
		void RenderBuffer();

		//Uses SDL to poll events such as keyboard input and the quit event 
		void PollEventsLoop();

		//A method that loops DisplayBuffer and RenderBuffer while managing SDL events and will terminate once SDL quit event is called 
		void RenderDisplayLoop();

		//Returns a thread of RenderDisplayLoop
		std::thread RenderDisplayLoopThread();
	};
}

