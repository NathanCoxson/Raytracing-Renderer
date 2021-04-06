#include "Raytracer.h"
#include "OBJ_Loader.h"

bool NC::Entity::setPos(glm::vec3 newPos)
{
	position = newPos;
	return true;
}

bool NC::Entity::setRot(glm::vec3 newRot)
{
	rotation = newRot;
	return true;
}

bool NC::Entity::setSca(glm::vec3 newSca)
{
	scale = newSca;
	return true;
}

NC::IntersectionTests::Result NC::IntersectionTests::rayEntity(Ray ray, Intersectable* entity, Intersectable::INTERSECTION_TYPE type)
{
	IntersectionTests::Result result;
	switch (type)
	{
	case Intersectable::INTERSECTION_TYPE::DEFAULT:
		return result;
	case Intersectable::INTERSECTION_TYPE::RAY_TO_SPHERE:
		return IntersectionTests::raySphere(ray, *(static_cast<Sphere*>(entity)));
	case Intersectable::INTERSECTION_TYPE::RAY_TO_PLANE:
		return IntersectionTests::rayPlane(ray, *(static_cast<Plane*>(entity)));
	case Intersectable::INTERSECTION_TYPE::RAY_TO_TRIANGLE:
		return IntersectionTests::rayTriangle(ray, *(static_cast<Triangle*>(entity)));
	case Intersectable::INTERSECTION_TYPE::RAY_TO_OBJECT:
		return IntersectionTests::rayModel(ray, *(static_cast<Model*>(entity)));
	default:
		return result;
	}
}

NC::IntersectionTests::Result NC::IntersectionTests::raySphere(Ray ray, Sphere sphere)
{
	Result result;
	result.hit = false;

	glm::vec3 l = sphere.position - ray.position;

	float tca = glm::dot(l, ray.direction);

	if (tca < 0) return result;

	float s = glm::dot(l, l) - (tca * tca);

	if (s > (sphere.radius * sphere.radius)) return result;

	float thc = sqrt((sphere.radius * sphere.radius) - s);

	float t0 = tca - thc;

	float t1 = tca + thc;
	
	if (t0 > t1) std::swap(t0, t1);

	if (t0 < 0)
	{
		t0 = t1;
		if (t1 < 0) return result;
	}

	glm::vec3 intersectionPoint = ray.position + t0 * ray.direction;

	glm::vec3 normal = (intersectionPoint - sphere.position) / glm::length(intersectionPoint - sphere.position);

	result.hit = true;

	result.normal = normal;

	result.length = t0;

	result.intersectionPoint = ray.position + t0 * ray.direction;

	return result;
}

NC::IntersectionTests::Result NC::IntersectionTests::rayPlane(Ray ray, Plane plane)
{
	Result result;
	result.hit = false;

	double denominator = glm::dot(plane.normalToPlane, ray.direction);

	if (denominator < 0.0) {
		float t = (glm::dot((plane.position - ray.position), plane.normalToPlane)) / (denominator);

		if (t > 0)
		{
			result.hit = true;
			result.length = t;
			result.normal = plane.normalToPlane;
			result.intersectionPoint = ray.position + t * ray.direction;
		}
	}

	return result;
}

NC::IntersectionTests::Result NC::IntersectionTests::rayTriangle(Ray ray, Triangle triangle)
{
	Result result;
	result.hit = false;

	Result boundingBoxResult = NC::IntersectionTests::rayAABB(ray, triangle.boundingBox);

	if (boundingBoxResult.hit != true) return result;

	glm::vec3 rayToVert0 = ray.position - triangle.vertices[0].position;
	glm::vec3 e1 = triangle.vertices[1].position - triangle.vertices[0].position;
	glm::vec3 e2 = triangle.vertices[2].position - triangle.vertices[0].position;

	glm::vec3 crossRayToVert0Edge1 = glm::cross(rayToVert0, e1);
	glm::vec3 crossRaydirEdge2 = glm::cross(ray.direction, e2);
	float dotEdge1CrossRaydirEdge2 = glm::dot(e1, crossRaydirEdge2);

	float u = glm::dot(rayToVert0, crossRaydirEdge2) / dotEdge1CrossRaydirEdge2;
	if (u < 0 || u > 1) return result;

	float v = glm::dot(ray.direction, crossRayToVert0Edge1) / dotEdge1CrossRaydirEdge2;
	if (v < 0 || u + v > 1) return result;

	float w = 1 - (u + v);

	float t = glm::dot(e2, crossRayToVert0Edge1) / dotEdge1CrossRaydirEdge2;

	if (t <= 0) return result;

	result.hit = true;
	result.length = t;
	result.intersectionPoint = ray.position + t * ray.direction;
	result.normal = glm::normalize(w * triangle.vertices[0].normal + u * triangle.vertices[1].normal + v * triangle.vertices[2].normal);
	return result;
}

NC::IntersectionTests::Result NC::IntersectionTests::rayAABB(Ray ray, AABB aabb)
{
	Result result;
	result.hit = false;

	glm::vec3 t1 = glm::vec3((aabb.minPoints.x - ray.position.x) / ray.direction.x, (aabb.minPoints.y - ray.position.y) / ray.direction.y, (aabb.minPoints.z - ray.position.z) / ray.direction.z);
	glm::vec3 t2 = glm::vec3((aabb.maxPoints.x - ray.position.x) / ray.direction.x, (aabb.maxPoints.y - ray.position.y) / ray.direction.y, (aabb.maxPoints.z - ray.position.z) / ray.direction.z);

	std::vector<glm::vec3> planes = { ray.position + ray.direction * t1.x, ray.position + ray.direction * t1.y, ray.position + ray.direction * t1.z, ray.position + ray.direction * t2.x, ray.position + ray.direction * t2.y, ray.position + ray.direction * t2.z };

	for (auto p : planes)
	{
		if (p.x > aabb.minPoints.x - 0.001f && p.x < aabb.maxPoints.x + 0.001f && p.y > aabb.minPoints.y - 0.001f && p.y < aabb.maxPoints.y + 0.001f && p.z > aabb.minPoints.z - 0.001f && p.z < aabb.maxPoints.z + 0.001f)
		{
			result.hit = true;
			result.intersectionPoint = p;
			result.length = glm::length(p);
			break;
		}
	}

	return result;
}

NC::IntersectionTests::Result NC::IntersectionTests::rayModel(Ray ray, Model object)
{
	Result shortestResult;
	shortestResult.length = 3e38f;

	Result boundingBoxResult = NC::IntersectionTests::rayAABB(ray, object.boundingBox);

	//std::cout << boundingBoxResult.hit << std::endl;

	if (boundingBoxResult.hit != true) return shortestResult;

	for (auto triangle : object.triangles)
	{
		Result result = IntersectionTests::rayTriangle(ray, triangle);
		if (result.hit)
		{
			if (result.length < shortestResult.length)
			{
				shortestResult = result;
			}
		}
	}

	return shortestResult;
}

bool NC::IntersectionTests::rayOctTreeNode(Ray ray, OctTreeNode* octTree)
{

	glm::vec3 octTreeMinPoints = octTree->position - octTree->extents;
	glm::vec3 octTreeMaxPoints = octTree->position + octTree->extents;

	glm::vec3 t1 = glm::vec3((octTreeMinPoints.x - ray.position.x) / ray.direction.x, (octTreeMinPoints.y - ray.position.y) / ray.direction.y, (octTreeMinPoints.z - ray.position.z) / ray.direction.z);
	glm::vec3 t2 = glm::vec3((octTreeMaxPoints.x - ray.position.x) / ray.direction.x, (octTreeMaxPoints.y - ray.position.y) / ray.direction.y, (octTreeMaxPoints.z - ray.position.z) / ray.direction.z);

	std::vector<glm::vec3> planes = { ray.position + ray.direction * t1.x, ray.position + ray.direction * t1.y, ray.position + ray.direction * t1.z, ray.position + ray.direction * t2.x, ray.position + ray.direction * t2.y, ray.position + ray.direction * t2.z };

	for (auto p : planes)
	{
		if (p.x > octTreeMinPoints.x - 0.001f && p.x < octTreeMaxPoints.x + 0.001f && p.y > octTreeMinPoints.y - 0.001f && p.y < octTreeMaxPoints.y + 0.001f && p.z > octTreeMinPoints.z - 0.001f && p.z < octTreeMaxPoints.z + 0.001f)
		{
			return true;
		}
	}

	return false;
}

bool NC::IntersectionTests::OctTreeBoxToAABB(OctTreeNode octTree, AABB aabb)
{
	glm::vec3 octTreeMinPoints = octTree.position - octTree.extents;
	glm::vec3 octTreeMaxPoints = octTree.position + octTree.extents;

	return (aabb.minPoints.x <= octTreeMaxPoints.x && aabb.maxPoints.x >= octTreeMinPoints.x) &&
		(aabb.minPoints.y <= octTreeMaxPoints.y && aabb.maxPoints.y >= octTreeMinPoints.y) &&
		(aabb.minPoints.z <= octTreeMaxPoints.z && aabb.maxPoints.z >= octTreeMinPoints.z);
}

NC::IntersectionTests::CollisionResult NC::IntersectionTests::ClosestCollisionWithRay(Ray ray, std::vector<Intersectable*> intersectables, std::vector<int> indexIgnores)
{

	struct EquivalentToIndex
	{
		const int index;
		EquivalentToIndex(int _index) : index(_index) {}
		bool operator()(int val) const { return val == index; }
	};

	CollisionResult collisionResult;
	Result shortestResult;
	float shortestLength = 3e38f;
	int index = -1;

	for (int i = 0; i < intersectables.size(); i++)
	{
		if (std::any_of(indexIgnores.begin(), indexIgnores.end(), EquivalentToIndex(i))) continue;

		IntersectionTests::Result result = IntersectionTests::rayEntity(ray, intersectables[i], intersectables[i]->intersectionType);
		if (result.hit)
		{
			if (result.length < shortestLength)
			{
				shortestLength = result.length;
				shortestResult = result;
				index = i;
			}
		}
	}


	collisionResult.result = shortestResult;
	collisionResult.intersectedObject = (index == -1)? nullptr : intersectables[index];

	return collisionResult;

}

NC::GraphicsRenderer::GraphicsRenderer(float _width, float _height)
{
	width = _width;
	height = _height;

	if (SDL_Init(SDL_INIT_VIDEO) < 0) throw std::runtime_error("UNABLE TO INITIALISE SDL");

	window = SDL_CreateWindow("NC Raytracer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN);

	if (window == NULL) throw std::runtime_error("UNABLE TO CREATE SDL WINDOW");

	surface = SDL_GetWindowSurface(window);

	SDL_ConvertSurfaceFormat(surface, SDL_PIXELFORMAT_RGBA32, 0);

	camera.setPos(glm::vec3(0.0f, 0.0f, 0.0f));

	screenBuffer = new glm::vec3 * [width];
	processBuffer = new glm::vec3 * [width];
	eraseBuffer = new glm::vec3 * [width];


	for (int i = 0; i < width; i++)
	{
		screenBuffer[i] = new glm::vec3[height];
		processBuffer[i] = new glm::vec3[height];
		eraseBuffer[i] = new glm::vec3[height];
	}

	for (uint32_t i = 0; i < width; i++)
	{
		for (uint32_t j = 0; j < height; j++)
		{
			screenBuffer[i][j] = glm::vec3(0.0f);
			processBuffer[i][j] = glm::vec3(0.0f);
			eraseBuffer[i][j] = glm::vec3(0.0f);
		}

	}

	keyMap['w'] = false;
	keyMap['a'] = false;
	keyMap['s'] = false;
	keyMap['d'] = false;
	keyMap['q'] = false;
	keyMap['e'] = false;
	keyMap['r'] = false;
	keyMap['f'] = false;
	keyMap['@'] = false;
	keyMap['1'] = false;
	keyMap['2'] = false;
	keyMap['3'] = false;
	keyMap['4'] = false;
	keyMap['5'] = false;
	keyMap['6'] = false;
	keyMap['7'] = false;
	keyMap['8'] = false;
	keyMap['9'] = false;
}

NC::GraphicsRenderer::~GraphicsRenderer()
{

	SDL_DestroyWindow(window);

	SDL_Quit();

	for (int i = 0; i < width; i++)
	{
		delete screenBuffer[i];
		delete processBuffer[i];
		delete eraseBuffer[i];
	}

	delete screenBuffer;
	delete processBuffer;
	delete eraseBuffer;

	for (size_t i = 0; i < intersectableEntities.size(); i++)
	{
		delete intersectableEntities[i];
	}
}

void NC::GraphicsRenderer::SetSDLSurfacePixel(SDL_Surface* surface, int x, int y, Uint32 colour)
{
	Uint8* pixel = (Uint8*)surface->pixels + (y * surface->pitch) + (x * sizeof(Uint32));
	*((Uint32*)pixel) = colour;
}

Uint32 NC::GraphicsRenderer::Vec3ToUint32(glm::vec3 value)
{
	Uint8 red, green, blue;
	red = (int)(std::min(std::max(value.r, 0.0f), 1.0f) * 255);
	green = (int)(std::min(std::max(value.g, 0.0f), 1.0f) * 255);
	blue = (int)(std::min(std::max(value.b, 0.0f), 1.0f) * 255);

	Uint32 rgb = (red << 16) + (green << 8) + (blue);
	return rgb;
}

glm::vec3 NC::GraphicsRenderer::DrawPixel(glm::vec2 pixelPos, Camera camera)
{
	float aspectRatio = width / height;

	glm::vec2 pixelNormPos = glm::vec2(((pixelPos.x + 0.5f) / width), (pixelPos.y + 0.5f) / height);

	glm::vec2 pixelRemappedPos = glm::vec2(((2.0f * pixelNormPos.x) - 1.0f) * aspectRatio, (1.0f - (2.0f * pixelNormPos.y)));;
	
	glm::vec3 cameraSpace = glm::vec3(pixelRemappedPos.x * fieldOfView, pixelRemappedPos.y * fieldOfView, -1.0f);

	glm::vec3 rayPosition = camera.position;

	glm::vec3 rayDirection = cameraSpace;

	rayDirection = glm::rotate(rayDirection, glm::radians(camera.rotation.x), glm::vec3(1, 0, 0));
	rayDirection = glm::rotate(rayDirection, glm::radians(camera.rotation.y), glm::vec3(0, 1, 0));
	rayDirection = glm::rotate(rayDirection, glm::radians(camera.rotation.z), glm::vec3(0, 0, 1));

	Ray ray = Ray(rayPosition, glm::normalize(rayDirection));

	IntersectionTests::CollisionResult collisionResult;

	std::vector<Intersectable*> intersectablesVector(intersectablePrimitives.begin(), intersectablePrimitives.end());

	if (useOctTree)
	{
		if (renderComplexGeometry)
		{
			std::set<Intersectable*> intersectables = octTree.GetAllPotentialIntersectables(ray);
			intersectablesVector.insert(intersectablesVector.end(), intersectables.begin(), intersectables.end());
		}

		collisionResult = IntersectionTests::ClosestCollisionWithRay(ray, intersectablesVector);
	}
	else
	{

		if (renderComplexGeometry) intersectablesVector.insert(intersectablesVector.end(), intersectableEntities.begin(), intersectableEntities.end());

		collisionResult = IntersectionTests::ClosestCollisionWithRay(ray, intersectablesVector);

	}

	if (collisionResult.result.hit == false) return glm::vec3(0.0f);

	glm::vec3 ambientColour = ambientLightIntensity; 
	glm::vec3 diffuseColour = glm::vec3(0.0f);
	glm::vec3 specularColour = glm::vec3(0.0f);

	glm::vec3 returnAmbient = glm::vec3(0.0f);

	if (shadingType == SHADING_TYPE::NO_PHONG_SHADING) return collisionResult.intersectedObject->colours.ambient;

	int amount = 0;
	for (auto areaLight : areaLights)
	{
		Ray lightRay;
		std::vector<glm::vec3> noiseySamples = areaLight.SampleLocationsWithNoise(sampleAmount.x, sampleAmount.y, sampleAmount.z);
		std::vector<glm::vec3> discreteSamples = areaLight.SampleLocations(sampleAmount.x, sampleAmount.y, sampleAmount.z);


		if (shadingType == SHADING_TYPE::SOFT_SHADOWS)
		{
			for (auto sample : noiseySamples)
			{
				lightRay.position = collisionResult.result.intersectionPoint + collisionResult.result.normal * 1e-5f;
				lightRay.direction = glm::normalize((areaLight.position + sample) - collisionResult.result.intersectionPoint);

				std::vector<bool> seeLightRays = {};

				std::vector<Intersectable*> intersectablesVector(intersectablePrimitives.begin(), intersectablePrimitives.end());

				if (useOctTree)
				{
					if (renderComplexGeometry)
					{
						std::set<Intersectable*> intersectables = octTree.GetAllPotentialIntersectables(lightRay);
						intersectablesVector.insert(intersectablesVector.end(), intersectables.begin(), intersectables.end());
					}
				}
				else
				{
					if (renderComplexGeometry) intersectablesVector.insert(intersectablesVector.end(), intersectableEntities.begin(), intersectableEntities.end());
				}

				for (int i = 0; i < intersectablesVector.size(); i++)
				{
					if (collisionResult.intersectedObject == intersectablesVector[i]) continue;

					IntersectionTests::Result result = IntersectionTests::rayEntity(lightRay, intersectablesVector[i], intersectablesVector[i]->intersectionType);
					seeLightRays.push_back(result.hit);
				}

				bool seeLight = !std::any_of(seeLightRays.begin(), seeLightRays.end(), [](bool v) {return v; });
				if (seeLight)
				{
					amount++;
					diffuseColour += ((areaLight.lightIntensity * glm::max(0.0f, glm::dot(glm::normalize((areaLight.position + sample) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))) * float(float(1.f) / float(noiseySamples.size())));
					specularColour += ((areaLight.lightIntensity * glm::pow(glm::max(0.0f, glm::dot(glm::normalize(camera.position - collisionResult.result.intersectionPoint), glm::reflect(-glm::normalize((areaLight.position + sample) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))), collisionResult.intersectedObject->colours.specularPower)) * float(float(1.f) / float(noiseySamples.size() * areaLights.size())));
				}
			}
		}
		else if (shadingType == SHADING_TYPE::HARD_SHADOWS)
		{
			glm::vec3 lightAveragePosition = glm::vec3(0.0f);
			for (auto sample : discreteSamples) lightAveragePosition += sample;
			lightAveragePosition /= discreteSamples.size();

			lightRay.position = collisionResult.result.intersectionPoint + collisionResult.result.normal * 1e-5f;
			lightRay.direction = glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint);

			std::vector<bool> seeLightRays = {};

			std::vector<Intersectable*> intersectablesVector(intersectablePrimitives.begin(), intersectablePrimitives.end());

			if (useOctTree)
			{
				if (renderComplexGeometry) 
				{ 
					std::set<Intersectable*> intersectables = octTree.GetAllPotentialIntersectables(lightRay);
					intersectablesVector.insert(intersectablesVector.end(), intersectables.begin(), intersectables.end()); 
				}
			}
			else
			{
				if (renderComplexGeometry) intersectablesVector.insert(intersectablesVector.end(), intersectableEntities.begin(), intersectableEntities.end());
			}

			for (int i = 0; i < intersectablesVector.size(); i++)
			{
				if (collisionResult.intersectedObject == intersectablesVector[i]) continue;

				IntersectionTests::Result result = IntersectionTests::rayEntity(lightRay, intersectablesVector[i], intersectablesVector[i]->intersectionType);
				seeLightRays.push_back(result.hit);
			}

			bool seeLight = !std::any_of(seeLightRays.begin(), seeLightRays.end(), [](bool v) {return v; });
			if (seeLight)
			{
				amount++;
				diffuseColour += ((areaLight.lightIntensity * glm::max(0.0f, glm::dot(glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))) * float(float(1.f) / float(areaLights.size())));
				specularColour += ((areaLight.lightIntensity * glm::pow(glm::max(0.0f, glm::dot(glm::normalize(camera.position - collisionResult.result.intersectionPoint), glm::reflect(-glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))), collisionResult.intersectedObject->colours.specularPower)) * float(float(1.f) / float(areaLights.size())));
			}
		}
		else if (shadingType == SHADING_TYPE::NO_SHADOWS)
		{
			glm::vec3 lightAveragePosition = glm::vec3(0.0f);
			for (auto sample : discreteSamples) lightAveragePosition += sample;
			lightAveragePosition /= discreteSamples.size();
			amount++;

			diffuseColour = ((areaLight.lightIntensity * glm::max(0.0f, glm::dot(glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))) * float(float(1.f) / float(areaLights.size())));
			specularColour = ((areaLight.lightIntensity * glm::pow(glm::max(0.0f, glm::dot(glm::normalize(camera.position - collisionResult.result.intersectionPoint), glm::reflect(-glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))), collisionResult.intersectedObject->colours.specularPower)) * float(float(1.f) / float(areaLights.size())));
		}
		else if (shadingType == SHADING_TYPE::RECURSIVE_REFLECTION)
		{
			glm::vec3 lightAveragePosition = glm::vec3(0.0f);
			for (auto sample : discreteSamples) lightAveragePosition += sample;
			lightAveragePosition /= discreteSamples.size();
			amount++;

			diffuseColour += ((areaLight.lightIntensity * glm::max(0.0f, glm::dot(glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))) * float(float(1.f) / float(areaLights.size())));
			specularColour += ((areaLight.lightIntensity * glm::pow(glm::max(0.0f, glm::dot(glm::normalize(camera.position - collisionResult.result.intersectionPoint), glm::reflect(-glm::normalize((areaLight.position + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))), collisionResult.intersectedObject->colours.specularPower)) * float(float(1.f) / float(areaLights.size())));
		}
	}
	if (amount == 0) return ambientColour;

	glm::vec3 returnColour = (ambientColour * collisionResult.intersectedObject->colours.ambient + diffuseColour * collisionResult.intersectedObject->colours.ambient + specularColour * collisionResult.intersectedObject->colours.specular);
	
	if (shadingType == SHADING_TYPE::RECURSIVE_REFLECTION)
	{
		Ray reflectedRay = Ray(collisionResult.result.intersectionPoint + collisionResult.result.normal * 1e-4f, ray.direction - 2 * glm::dot(glm::normalize(ray.direction), glm::normalize(collisionResult.result.normal)) * collisionResult.result.normal);

		glm::vec3 objectColour = ((1.0f - collisionResult.intersectedObject->colours.totalReflection) * collisionResult.intersectedObject->colours.ambient + collisionResult.intersectedObject->colours.totalReflection * TraceReflectiveRay(reflectedRay, collisionResult.intersectedObject->colours.specular, 3, 0));
		
		returnColour = (ambientColour * objectColour + diffuseColour * ((1.0f - collisionResult.intersectedObject->colours.totalReflection) * collisionResult.intersectedObject->colours.ambient) + specularColour * collisionResult.intersectedObject->colours.specular);
	}

	if (HDR) return returnColour / (returnColour + glm::vec3(1.0f));
	else return returnColour;
}

std::thread NC::GraphicsRenderer::RtnDrawPixelThread(glm::vec2 pixelPos, Camera camera)
{
	return std::thread([=] {DrawPixelThread(pixelPos, camera); });
}

std::thread NC::GraphicsRenderer::RtnDrawHorizontalPixelsThread(int pixelY, int amountOfLines, int width, Camera camera)
{
	return std::thread([=] {DrawHorizontalPixelsThread(pixelY, amountOfLines, width, camera); });

}

void NC::GraphicsRenderer::DrawPixelThread(glm::vec2 pixelPos, Camera camera)
{
	glm::vec3 val = DrawPixel(pixelPos, camera);
	processBuffer[int(pixelPos.x)][int(pixelPos.y)] = val;
}

void NC::GraphicsRenderer::DrawHorizontalPixelsThread(int pixelY, int amountOfLines, int width, Camera camera)
{
	auto startTime = std::chrono::steady_clock::now();
	for (int y = pixelY; y < pixelY + amountOfLines; y++)
	{
		for (int x = 0; x < width; x++)
		{
			glm::vec3 val = DrawPixel(glm::vec2(x, y), camera);
			
			processBuffer[int(x)][int(y)] = val;
		}
	}
}

bool NC::GraphicsRenderer::CreateSphere(float radius, LightProperties colour, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale)
{
	Sphere* sphere = new Sphere;
	sphere->radius = radius;
	sphere->colours = colour;
	sphere->position = position;
	sphere->rotation = rotation;
	sphere->scale = scale;

	sphere->boundingBox = AABB();
	sphere->boundingBox.minPoints = position - glm::vec3(radius);
	sphere->boundingBox.maxPoints = position + glm::vec3(radius);
	
	intersectablePrimitives.push_back(sphere);
	return true;
}

bool NC::GraphicsRenderer::CreatePlane(glm::vec3 normal, LightProperties colour, glm::vec3 position)
{
	Plane* plane = new Plane;

	plane->normalToPlane = normal;
	plane->colours = colour;
	plane->position = position;
	plane->boundingBox = AABB();
	plane->boundingBox.minPoints = position - glm::vec3(100.0f);
	plane->boundingBox.maxPoints = position + glm::vec3(100.0f);

	intersectablePrimitives.push_back(plane);
	return true;
}

bool NC::GraphicsRenderer::CreateTriangle(Vertex vertexA, Vertex vertexB, Vertex vertexC, LightProperties colour)
{
	Triangle* triangle = new Triangle({ vertexA, vertexB, vertexC });
	triangle->colours = colour;

	intersectableEntities.push_back(triangle);
	return true;
}

bool NC::GraphicsRenderer::CreateObject(std::string filePath, LightProperties colour, glm::vec3 position, glm::vec3 rotation, glm::vec3 scale)
{

	objl::Loader loader;
	bool result = loader.LoadFile(filePath.c_str());

	for (size_t s = 0; s < loader.LoadedMeshes.size(); s++)
	{

		Model *model = new Model;

		glm::vec3 minPoints = glm::vec3(0.0f);
		glm::vec3 maxPoints = glm::vec3(0.0f);

		model->boundingBox = AABB();

		std::vector<Triangle> triangles;
		
		for (size_t f = 0; f < (loader.LoadedMeshes[s].Indices.size()); f += 3)
		{

			std::vector<Vertex> triangleVertices = {};

			for (size_t v = 0; v < 3; v++)
			{

				objl::Vertex objlVertex = loader.LoadedMeshes[s].Vertices[loader.LoadedMeshes[s].Indices[f + v]];
				Vertex vertex;

				vertex.position = position + glm::vec3(objlVertex.Position.X, objlVertex.Position.Y, objlVertex.Position.Z);
				vertex.normal = glm::vec3(objlVertex.Normal.X, objlVertex.Normal.Y, objlVertex.Normal.Z);
				vertex.texCoord = glm::vec2(objlVertex.TextureCoordinate.X, objlVertex.TextureCoordinate.Y);

				if (minPoints == glm::vec3(0.0f))
				{
					minPoints = vertex.position;
					maxPoints = vertex.position;
				}

				triangleVertices.push_back(vertex);

				minPoints = glm::vec3(std::min(vertex.position.x, minPoints.x), std::min(vertex.position.y, minPoints.y), std::min(vertex.position.z, minPoints.z));
				maxPoints = glm::vec3(std::max(vertex.position.x, maxPoints.x), std::max(vertex.position.y, maxPoints.y), std::max(vertex.position.z, maxPoints.z));
			}

			Triangle triangle = Triangle(triangleVertices);
			triangle.colours = colour;

			triangles.push_back(triangle);

		}

		model->colours = colour;
		model->position = position;
		model->rotation = rotation;
		model->scale = scale;
		model->triangles = triangles;
		model->boundingBox.minPoints = minPoints;
		model->boundingBox.maxPoints = maxPoints;

		intersectableEntities.push_back(model);
	}

	return true;
}

bool NC::GraphicsRenderer::CreatePointLight(glm::vec3 intensity, glm::vec3 position)
{
	LightProperties properties;
	properties.ambient = glm::vec3(1.0f);
	properties.diffuse = glm::vec3(1.0f);
	properties.specular = glm::vec3(1.0f);
	PointLight light(intensity, properties);
	light.position = position;

	pointLights.push_back(light);
	return true;
}

bool NC::GraphicsRenderer::CreateAreaLight(glm::vec3 intensity, glm::vec3 position, glm::vec3 startPos, glm::vec3 endPos)
{
	LightProperties properties;
	properties.ambient = glm::vec3(1.0f);
	properties.diffuse = glm::vec3(1.0f);
	properties.specular = glm::vec3(1.0f);
	AreaLight areaLight(intensity, properties, startPos, endPos);
	areaLight.position = position;

	areaLights.push_back(areaLight);
	return true;
}

glm::vec3 NC::GraphicsRenderer::TraceReflectiveRay(Ray ray, glm::vec3 specularCoefficient, int maxDepth, int depth)
{
	if (specularCoefficient == glm::vec3(0.0f)) return glm::vec3(0.0f);

	IntersectionTests::CollisionResult collisionResult;

	std::vector<Intersectable*> intersectablesVector(intersectablePrimitives.begin(), intersectablePrimitives.end());

	if (useOctTree)
	{

		std::set<Intersectable*> intersectables = octTree.GetAllPotentialIntersectables(ray);

		if (renderComplexGeometry) intersectablesVector.insert(intersectablesVector.end(), intersectables.begin(), intersectables.end());

		collisionResult = IntersectionTests::ClosestCollisionWithRay(ray, intersectablesVector);
	}
	else
	{
		
		if (renderComplexGeometry) intersectablesVector.insert(intersectablesVector.end(), intersectableEntities.begin(), intersectableEntities.end());

		collisionResult = IntersectionTests::ClosestCollisionWithRay(ray, intersectablesVector);
	}

	if (collisionResult.result.hit != true) return glm::vec3(0.0f);

	glm::vec3 diffuseColour = glm::vec3(0.0f);
	glm::vec3 specularColour = glm::vec3(0.0f);

	size_t lightSize = areaLights.size();

	for (int i = 0; i < lightSize; i++)
	{
		areaLightsLock.lock();
		glm::vec3 lightIntensity = areaLights[i].lightIntensity;

		glm::vec3 lightPosition = areaLights[i].position;

		glm::vec3 col = collisionResult.intersectedObject->colours.ambient;

		LightProperties lightProperties = collisionResult.intersectedObject->colours;

		std::vector<glm::vec3> samples = areaLights[i].SampleLocationsWithNoise(sampleAmount.x, sampleAmount.y, sampleAmount.z);

		areaLightsLock.unlock();

		glm::vec3 lightAveragePosition = glm::vec3(0.0f);
		for (auto sample : samples) lightAveragePosition += sample;
		lightAveragePosition /= samples.size();

		diffuseColour += ((lightIntensity * glm::max(0.0f, glm::dot(glm::normalize((lightPosition + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))) * float(float(1.f) / float(lightSize)));
		specularColour += ((lightIntensity * glm::pow(glm::max(0.0f, glm::dot(glm::normalize(camera.position - collisionResult.result.intersectionPoint), glm::reflect(-glm::normalize((lightPosition + lightAveragePosition) - collisionResult.result.intersectionPoint), glm::normalize(collisionResult.result.normal)))), lightProperties.specularPower)) * float(float(1.f) / float(lightSize)));
	}

	Ray reflectedRay = Ray(collisionResult.result.intersectionPoint + collisionResult.result.normal * 1e-4f, ray.direction - 2 * glm::dot(glm::normalize(ray.direction), glm::normalize(collisionResult.result.normal)) * collisionResult.result.normal);

	glm::vec3 returnColour;

	if (maxDepth > depth)
	{
		returnColour = (1.0f - collisionResult.intersectedObject->colours.totalReflection) * collisionResult.intersectedObject->colours.ambient + collisionResult.intersectedObject->colours.totalReflection * TraceReflectiveRay(reflectedRay, collisionResult.intersectedObject->colours.specular, maxDepth, depth + 1);
		returnColour = returnColour + diffuseColour * ((1.0f - collisionResult.intersectedObject->colours.totalReflection) * collisionResult.intersectedObject->colours.ambient) + specularColour * collisionResult.intersectedObject->colours.specular;
	}
	else
	{
		returnColour = collisionResult.intersectedObject->colours.ambient + diffuseColour * ((1.0f - collisionResult.intersectedObject->colours.specular) * collisionResult.intersectedObject->colours.ambient) + specularColour * collisionResult.intersectedObject->colours.specular;
	}

	return returnColour;
}

void NC::GraphicsRenderer::DisplayBuffer()
{
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < height; j++)
		{
			SetSDLSurfacePixel(surface, i, j, Vec3ToUint32(screenBuffer[i][j]));
		}
	}

	SDL_UpdateWindowSurface(window);
}

void NC::GraphicsRenderer::RenderBuffer()
{

	auto startTime = std::chrono::steady_clock::now();

	if (multiThreading)
	{
	
		std::queue<std::thread> threads = {};

		int stride = 16;

		for (int i = 0; i < height; i+= stride)
		{
			if ((i + stride) > height)
			{
				stride = height - i;
			}
			threads.push(RtnDrawHorizontalPixelsThread(i, stride, width, camera));

		}

		while (threads.size() > 0)
		{
			threads.front().join();
			threads.pop();
		}

	}
	else
	{
		for (int i = 0; i < width; i++)
		{
			for (int j = 0; j < height; j++)
			{
				processBuffer[i][j] = DrawPixel(glm::vec2(i, j), camera);
			}
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

	std::cout << " - - - - - ----------------- - - - - - " << std::endl;
	std::cout << "SHADING:         ";
	switch(shadingType)
	{
	case SHADING_TYPE::NO_PHONG_SHADING:
		std::cout << "NO PHONG SHADING" << std::endl;
		break;
	case SHADING_TYPE::NO_SHADOWS:
		std::cout << "PHONG SHADING" << std::endl;
		break;
	case SHADING_TYPE::HARD_SHADOWS:
		std::cout << "PHONG SHADING WITH HARD SHADOWS" << std::endl;
		break;
	case SHADING_TYPE::SOFT_SHADOWS:
		std::cout << "PHONG SHADING WITH SOFT SHADOWS" << std::endl;
		break;
	case SHADING_TYPE::RECURSIVE_REFLECTION:
		std::cout << "PHONG SHADING WITH RECURSIVE REFLECTION" << std::endl;
		break;
	}
	std::cout << "OCTTREE:         " << (useOctTree ? "TRUE" : "FALSE") << std::endl;
	std::cout << "MULTITHREADING:  " << (multiThreading ? "TRUE" : "FALSE") << std::endl;
	std::cout << "COMPLEX GEOMETRY:" << (renderComplexGeometry ? "TRUE" : "FALSE") << std::endl;
	std::cout << "TOTAL TIME ELAPSED: " << (elapsed.count() / 1000000.0f) << " SECONDS" << std::endl;

}

void NC::GraphicsRenderer::RenderDisplayLoop()
{
	while (true)
	{
		std::thread render = std::thread([=] {RenderBuffer();});
		std::thread display = std::thread([=] {DisplayBuffer(); });

		render.join();
		display.join();

		screenBufferLock.lock();
		processBufferLock.lock();

		glm::vec3** renderStore = processBuffer;
		glm::vec3** screenStore = screenBuffer;

		processBuffer = screenStore;
		screenBuffer = renderStore;

		screenBufferLock.unlock();
		processBufferLock.unlock();

		keyMapLock.lock();

		if (keyMap['@'])
		{
			keyMapLock.unlock();
			pollEventsLock.lock();
			pollEvents = false;
			pollEventsLock.unlock();
			break;
		}

		if (keyMap['w'])
		{
			glm::vec3 direction = glm::rotate(glm::vec3(0.0f, 0.0f, -1.0f), glm::radians(camera.rotation.x), glm::vec3(1, 0, 0));
			direction = glm::rotate(direction, glm::radians(camera.rotation.y), glm::vec3(0, 1, 0));
			direction = glm::rotate(direction, glm::radians(camera.rotation.z), glm::vec3(0, 0, 1));

			camera.position += direction;
			keyMap['w'] = false;
		}
		if (keyMap['s'])
		{
			glm::vec3 direction = glm::rotate(glm::vec3(0.0f, 0.0f, 1.0f), glm::radians(camera.rotation.x), glm::vec3(1, 0, 0));
			direction = glm::rotate(direction, glm::radians(camera.rotation.y), glm::vec3(0, 1, 0));
			direction = glm::rotate(direction, glm::radians(camera.rotation.z), glm::vec3(0, 0, 1));

			camera.position += direction;
			keyMap['s'] = false;
		}
		if (keyMap['q']) camera.position += glm::vec3( 0.0f,  1.0f,  0.0f); keyMap['q'] = false;
		if (keyMap['e']) camera.position += glm::vec3( 0.0f, -1.0f,  0.0f); keyMap['e'] = false;
		if (keyMap['a']) camera.rotation += glm::vec3( 0.0f,  5.0f,  0.0f); keyMap['a'] = false;
		if (keyMap['d']) camera.rotation += glm::vec3( 0.0f, -5.0f,  0.0f); keyMap['d'] = false;
		if (keyMap['r']) camera.rotation += glm::vec3( 5.0f,  0.0f,  0.0f); keyMap['r'] = false;
		if (keyMap['f']) camera.rotation += glm::vec3(-5.0f,  0.0f,  0.0f); keyMap['f'] = false;


		if (keyMap['1']) shadingType = SHADING_TYPE::NO_PHONG_SHADING; keyMap['1'] = false;
		if (keyMap['2']) shadingType = SHADING_TYPE::NO_SHADOWS; keyMap['2'] = false;
		if (keyMap['3']) shadingType = SHADING_TYPE::HARD_SHADOWS; keyMap['3'] = false;
		if (keyMap['4']) shadingType = SHADING_TYPE::SOFT_SHADOWS; keyMap['4'] = false;
		if (keyMap['5']) shadingType = SHADING_TYPE::RECURSIVE_REFLECTION; keyMap['5'] = false;
		if (keyMap['6']) useOctTree = useOctTree ? false : true; keyMap['6'] = false;
		if (keyMap['7']) multiThreading = multiThreading ? false : true; keyMap['7'] = false;
		if (keyMap['8']) renderComplexGeometry = renderComplexGeometry ? false : true; keyMap['8'] = false;

		keyMapLock.unlock();
	}

}

std::thread NC::GraphicsRenderer::RenderDisplayLoopThread()
{
	return std::thread([=] {RenderDisplayLoop(); });
}

void NC::GraphicsRenderer::PollEventsLoop()
{
	while (true)
	{

		pollEventsLock.lock();
		if (!pollEvents)
		{
			pollEventsLock.unlock();
			break;
		}
		pollEventsLock.unlock();

		SDL_Event event;
		if (SDL_PollEvent(&event) != 0)
		{

			if (event.type == SDL_QUIT)
			{
				keyMapLock.lock();
				keyMap['@'] = true;
				keyMapLock.unlock();
			}
			else if (event.type == SDL_KEYDOWN)
			{
				keyMapLock.lock();
				switch (event.key.keysym.sym)
				{
				case SDLK_1:
					(keyMap)['1'] = true;
					break;
				case SDLK_2:
					(keyMap)['2'] = true;
					break;
				case SDLK_3:
					(keyMap)['3'] = true;
					break;
				case SDLK_4:
					(keyMap)['4'] = true;
					break;
				case SDLK_5:
					(keyMap)['5'] = true;
					break;
				case SDLK_6:
					(keyMap)['6'] = true;
					break;
				case SDLK_7:
					(keyMap)['7'] = true;
					break;
				case SDLK_8:
					(keyMap)['8'] = true;
					break;
				case SDLK_9:
					(keyMap)['9'] = true;
					break;
				case SDLK_w:
					(keyMap)['w'] = true;
					break;
				case SDLK_a:
					(keyMap)['a'] = true;
					break;
				case SDLK_s:
					(keyMap)['s'] = true;
					break;
				case SDLK_d:
					(keyMap)['d'] = true;
					break;
				case SDLK_q:
					(keyMap)['q'] = true;
					break;
				case SDLK_e:
					(keyMap)['e'] = true;
					break;
				case SDLK_r:
					(keyMap)['r'] = true;
					break;
				case SDLK_f:
					(keyMap)['f'] = true;
					break;
				}
				keyMapLock.unlock();
			}
		}
	}
}

NC::Sphere::Sphere()
{
	intersectionType = INTERSECTION_TYPE::RAY_TO_SPHERE;
}

NC::Plane::Plane()
{
	intersectionType = INTERSECTION_TYPE::RAY_TO_PLANE;
}

NC::Triangle::Triangle()
{
	intersectionType = INTERSECTION_TYPE::RAY_TO_TRIANGLE;
}

NC::Triangle::Triangle(std::vector<Vertex> _vertices)
{
	intersectionType = INTERSECTION_TYPE::RAY_TO_TRIANGLE;
	vertices = _vertices;

	glm::vec3 minPoints = _vertices[0].position;
	glm::vec3 maxPoints = _vertices[0].position;

	boundingBox = AABB();

	for (auto vertex : vertices)
	{
		minPoints = glm::vec3(std::min(vertex.position.x, minPoints.x), std::min(vertex.position.y, minPoints.y), std::min(vertex.position.z, minPoints.z));
		maxPoints = glm::vec3(std::max(vertex.position.x, maxPoints.x), std::max(vertex.position.y, maxPoints.y), std::max(vertex.position.z, maxPoints.z));
	}

	boundingBox.minPoints = minPoints;
	boundingBox.maxPoints = maxPoints;
}

NC::AABB::AABB()
{
	intersectionType = INTERSECTION_TYPE::RAY_TO_AABB;
}

NC::Model::Model()
{
	intersectionType = INTERSECTION_TYPE::RAY_TO_OBJECT;
}

std::vector<glm::vec3> NC::AreaLight::SampleLocations(int sampleAmountX, int sampleAmountY, int sampleAmountZ)
{
	std::vector<glm::vec3> samples;
	if (sampleAmountX < 1 || sampleAmountY < 1 || sampleAmountZ < 1) return samples;

	glm::vec3 distance = relativeEndPos - relativeStartPos;
	glm::vec3 finalDistance = relativeEndPos - relativeStartPos;

	if (distance.x == 0 || distance.y == 0 || distance.z == 0)
	{
		if (distance.x == 0) finalDistance.x = 1;
		if (distance.y == 0) finalDistance.y = 1;
		if (distance.z == 0) finalDistance.z = 1;
	}

	glm::vec3 amount = distance / glm::vec3(sampleAmountX+1, sampleAmountY+1, sampleAmountZ+1);

	if (distance.x == 0) amount.x = 0;
	if (distance.y == 0) amount.y = 0;
	if (distance.z == 0) amount.z = 0;

	for (size_t i = 0; i < sampleAmountX; i++)
	{
		for (size_t j = 0; j < sampleAmountY; j++)
		{
			for (size_t k = 0; k < sampleAmountZ; k++)
			{
				samples.push_back(glm::vec3(relativeStartPos.x + amount.x * (i + 1), relativeStartPos.y + amount.y * (j + 1), relativeStartPos.z + amount.z * (k + 1)));
			}
		}
	}

	return samples;
}

std::vector<glm::vec3> NC::AreaLight::SampleLocationsWithNoise(int sampleAmountX, int sampleAmountY, int sampleAmountZ)
{
	std::vector<glm::vec3> samples;
	if (sampleAmountX < 1 || sampleAmountY < 1 || sampleAmountZ < 1) return samples;

	glm::vec3 distance = relativeEndPos - relativeStartPos;
	glm::vec3 finalDistance = relativeEndPos - relativeStartPos;

	if (distance.x == 0 || distance.y == 0 || distance.z == 0)
	{
		if (distance.x == 0) finalDistance.x = 1;
		if (distance.y == 0) finalDistance.y = 1;
		if (distance.z == 0) finalDistance.z = 1;
	}

	glm::vec3 amount = distance / glm::vec3(sampleAmountX + 1, sampleAmountY + 1, sampleAmountZ + 1);

	if (distance.x == 0) amount.x = 0;
	if (distance.y == 0) amount.y = 0;
	if (distance.z == 0) amount.z = 0;

	for (size_t i = 0; i < sampleAmountX; i++)
	{
		for (size_t j = 0; j < sampleAmountY; j++)
		{
			for (size_t k = 0; k < sampleAmountZ; k++)
			{
				srand((unsigned int)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
				float xNoise = (((rand() % 101) - 50.0f) / 100.0f);
				float yNoise = (((rand() % 101) - 50.0f) / 100.0f);
				float zNoise = (((rand() % 101) - 50.0f) / 100.0f);

				samples.push_back(glm::vec3(relativeStartPos.x + amount.x * (i + 1) + amount.x * xNoise, relativeStartPos.y + amount.y * (j + 1) + amount.y * yNoise, relativeStartPos.z + amount.z * (k + 1) + amount.z * zNoise));
			}
		}
	}

	return samples;
}

NC::OctTreeNode::OctTreeNode(glm::vec3 _position, glm::vec3 _extents, int _maxDepth, int _splitAmount, int _currentDepth)
{
	position = _position;
	extents = _extents;
	maxDepth = _maxDepth;
	splitAmount = _splitAmount;
	currentDepth = _currentDepth;
}

NC::OctTreeNode::~OctTreeNode()
{
	if (!endNode)
	{
		for (int i = 0; i < 8; i++)
		{
			delete nodes[i];
		}
	}
}

bool NC::OctTreeNode::FillOctTree(std::vector<Intersectable*> intersectables)
{
	for (auto intersectable : intersectables)
	{
		if (intersectable->intersectionType == Intersectable::INTERSECTION_TYPE::RAY_TO_SPHERE)
		{
			if (IntersectionTests::OctTreeBoxToAABB(*this, ((Sphere*)intersectable)->boundingBox))containedIntersectables.insert(intersectable);
		}
		else if (intersectable->intersectionType == Intersectable::INTERSECTION_TYPE::RAY_TO_PLANE)
		{
			if (IntersectionTests::OctTreeBoxToAABB(*this, ((Plane*)intersectable)->boundingBox))containedIntersectables.insert(intersectable);
		}
		else if (intersectable->intersectionType == Intersectable::INTERSECTION_TYPE::RAY_TO_TRIANGLE)
		{
			if (IntersectionTests::OctTreeBoxToAABB(*this, ((Triangle*)intersectable)->boundingBox)) containedIntersectables.insert(intersectable);
		}
		else if (intersectable->intersectionType == Intersectable::INTERSECTION_TYPE::RAY_TO_OBJECT)
		{
			for (int i = 0; i < ((Model*)intersectable)->triangles.size(); i++)
			{
				if (IntersectionTests::OctTreeBoxToAABB(*this, (&((Model*)intersectable)->triangles[i])->boundingBox))
				{
					Intersectable* inst = ((Intersectable*)(&((Model*)intersectable)->triangles[i]));
					containedIntersectables.insert(inst);
				}
			}
		}
	}

	if (int(containedIntersectables.size()) >= splitAmount && maxDepth > currentDepth)
	{

		endNode = false;

		glm::vec3 newExtents = extents / 2.0f;

		nodes.push_back(new OctTreeNode(glm::vec3(position.x - (newExtents.x), position.y + (newExtents.y), position.z + (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x + (newExtents.x), position.y + (newExtents.y), position.z + (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x - (newExtents.x), position.y - (newExtents.y), position.z + (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x + (newExtents.x), position.y - (newExtents.y), position.z + (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x - (newExtents.x), position.y + (newExtents.y), position.z - (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x + (newExtents.x), position.y + (newExtents.y), position.z - (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x - (newExtents.x), position.y - (newExtents.y), position.z - (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));
		nodes.push_back(new OctTreeNode(glm::vec3(position.x + (newExtents.x), position.y - (newExtents.y), position.z - (newExtents.z)), newExtents, maxDepth, splitAmount, currentDepth + 1));

		for (int i = 0; i < 8; i++)
		{
			(nodes[i])->FillOctTree(intersectables);
		}

	}
	else
	{
		endNode = true;
	}

	return true;
}

std::set<NC::Intersectable*> NC::OctTreeNode::GetAllPotentialIntersectables(Ray ray)
{

	std::set<Intersectable*> intersectables = {};

	if (!IntersectionTests::rayOctTreeNode(ray, this)) return intersectables;

	if (this->endNode)
	{

		return containedIntersectables;
	}
	else
	{

		for (int i = 0; i < 8; i++)
		{

			std::set<Intersectable*> intersected = (nodes[i])->GetAllPotentialIntersectables(ray);
			intersectables.insert(intersected.begin(), intersected.end());
		}
	}

	return intersectables;
}
