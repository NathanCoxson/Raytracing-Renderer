#pragma once
#include "Raytracer.h"

int main()
{
	SDL_SetMainReady();

	NC::GraphicsRenderer renderer(640, 480);

	NC::LightProperties property1;
	property1.ambient = glm::vec3(1.0f, 0.32f, 0.36f);
	property1.diffuse = glm::vec3(0.8f);
	property1.specular = glm::vec3(0.8f);
	property1.specularPower = 32.0f;
	property1.totalReflection = 0.5f;
	renderer.CreateSphere(4.0f, property1, glm::vec3(0.0f, 0.0f, -20.0f), glm::vec3(0.0f), glm::vec3(1.0f));

	NC::LightProperties property2;
	property2.ambient = glm::vec3(0.9f, 0.76f, 0.46f);
	property2.diffuse = glm::vec3(0.8f);
	property2.specular = glm::vec3(0.8f);
	property2.specularPower = 32.0f;
	property2.totalReflection = 0.0f;
	renderer.CreateSphere(2.0f, property2, glm::vec3(5.0f, -1.0f, -15.0f), glm::vec3(0.0f), glm::vec3(1.0f));

	NC::LightProperties property3;
	property3.ambient = glm::vec3(0.65f, 0.77f, 0.97f);
	property3.diffuse = glm::vec3(0.8f);
	property3.specular = glm::vec3(0.9f);
	property3.specularPower = 32.0f;
	property3.totalReflection = 0.0f;
	renderer.CreateSphere(3.0f, property3, glm::vec3(5.0f, 0.0f, -25.0f), glm::vec3(0.0f), glm::vec3(1.0f));

	NC::LightProperties property4;
	property4.ambient = glm::vec3(0.9f);
	property4.diffuse = glm::vec3(0.8f);
	property4.specular = glm::vec3(1.0f);
	property4.specularPower = 32.0f;
	property4.totalReflection = 1.0f;
	renderer.CreateSphere(3.0f, property4, glm::vec3(-5.5f, 0.0f, -15.0f), glm::vec3(0.0f), glm::vec3(1.0f));

	NC::LightProperties property5;
	property5.ambient = glm::vec3(0.8f);
	property5.diffuse = glm::vec3(0.8f);
	property5.specular = glm::vec3(0.8f);
	property5.specularPower = 32.0f;
	property5.totalReflection = 0.0f;
	renderer.CreatePlane(glm::vec3(0.0f, 1.0f, 0.0f), property5, glm::vec3(0.0f, -8.0f, 0.0f));

	NC::LightProperties property7;
	property7.ambient = glm::vec3(0.5f, 0.5f, 0.0f);
	property7.diffuse = property7.ambient;
	property7.specular = glm::vec3(0.8f);
	property7.specularPower = 32.0f;

	renderer.CreateObject("decimatedMonkey.obj", property7, glm::vec3(-6.0f, 0.0f, -9.0f), glm::vec3(0.0f), glm::vec3(1.0f));
	renderer.CreateObject("decimatedMonkey.obj", property7, glm::vec3(-2.0f, 3.0f, -9.0f), glm::vec3(0.0f), glm::vec3(1.0f));

	renderer.CreateAreaLight(glm::vec3(1.0f), glm::vec3(6.0f, 10.0f, 1.0f), glm::vec3(-1.0f, -1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f));
	
	renderer.octTree = NC::OctTreeNode(glm::vec3(0.0f, 5.0f, 0.0f), glm::vec3(30.0f, 12.0f, 30.0f), 8, 3);

	renderer.octTree.FillOctTree(renderer.intersectableEntities);

	renderer.useOctTree = true;

	std::thread renderThread = renderer.RenderDisplayLoopThread();

	renderer.camera.position = glm::vec3(0.0f, 0.0f, 5.0f);

	renderer.PollEventsLoop();
	renderThread.join();
}
