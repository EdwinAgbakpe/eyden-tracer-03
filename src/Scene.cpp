#include <iostream>
#include "Scene.h"
#include "ShaderEyelight.h"
#include "ShaderPhong.h"
#include "PrimTriangle.h"


void CScene::ParseOBJ(const std::string& fileName)
{
	std::cout << "Parsing OBJFile : " << fileName << std::endl;	
	std::shared_ptr<IShader> pShader = std::make_shared<CShaderEyelight>(RGB(1, 1, 1));
	// --- PUT YOUR CODE HERE ---
	std::vector<Vec3f> vertices;
	std::string line, c;
	std::ifstream in;
	in.open (fileName, std::ios::in);
	if (!(in.is_open())){
			std::cerr << "Cannot open " << fileName << std::endl;
			exit(1);

	}

	while (getline(in, line)){
		std::istringstream iss(line);
        iss >> c;
		if (c == "v") {
			double x, y, z;
			iss >> c;
			x = std::stod(c);
			iss >> c;
			y = std::stod(c);
			iss >> c;
			z = std::stod(c);
			vertices.push_back(Vec3f(x * 100.0f,y * 100.0f, z * 100.0f));
		}
		else if (c == "f") {
			int v1_nr, v2_nr, v3_nr;
			iss >> c;
			v1_nr = std::stoi(c);
		
			iss >> c;
			v2_nr = std::stoi(c);
			
			iss >> c;
			v3_nr = std::stoi(c);
			
			Add(std::make_shared<CPrimTriangle>(vertices[v1_nr - 1], vertices[v2_nr - 1], vertices[v3_nr - 1], pShader));
		}

	}
	in.close();
	std::cout << "Finished Parsing" << std::endl;
}

