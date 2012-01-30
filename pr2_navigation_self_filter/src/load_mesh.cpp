/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <set>
#include "pr2_navigation_self_filter/shapes.h"

// \author Ioan Sucan ;  based on stl_to_mesh 

namespace shapes
{

    namespace detail
    {
	struct myVertex
	{
	    tf::Vector3    point;
	    unsigned int index;
	};
	
	struct ltVertexValue
	{
	    bool operator()(const myVertex &p1, const myVertex &p2) const
	    {
		const tf::Vector3 &v1 = p1.point;
		const tf::Vector3 &v2 = p2.point;
		if (v1.getX() < v2.getX())
		    return true;
		if (v1.getX() > v2.getX())
		    return false;
		if (v1.getY() < v2.getY())
		    return true;
		if (v1.getY() > v2.getY())
		    return false;
		if (v1.getZ() < v2.getZ())
		    return true;
		return false;
	    }
	};
	
	struct ltVertexIndex
	{
	    bool operator()(const myVertex &p1, const myVertex &p2) const
	    {
		return p1.index < p2.index;
	    }
	};
    }
    
    shapes::Mesh* createMeshFromVertices(const std::vector<tf::Vector3> &vertices, const std::vector<unsigned int> &triangles)
    {
	unsigned int nt = triangles.size() / 3;
	shapes::Mesh *mesh = new shapes::Mesh(vertices.size(), nt);
	for (unsigned int i = 0 ; i < vertices.size() ; ++i)
	{
	    mesh->vertices[3 * i    ] = vertices[i].getX();
	    mesh->vertices[3 * i + 1] = vertices[i].getY();
	    mesh->vertices[3 * i + 2] = vertices[i].getZ();
	}
	
	std::copy(triangles.begin(), triangles.end(), mesh->triangles);
	
	// compute normals 
	for (unsigned int i = 0 ; i < nt ; ++i)
	{
	    tf::Vector3 s1 = vertices[triangles[i * 3    ]] - vertices[triangles[i * 3 + 1]];
	    tf::Vector3 s2 = vertices[triangles[i * 3 + 1]] - vertices[triangles[i * 3 + 2]];
	    tf::Vector3 normal = s1.cross(s2);
	    normal.normalize();
	    mesh->normals[3 * i    ] = normal.getX();
	    mesh->normals[3 * i + 1] = normal.getY();
	    mesh->normals[3 * i + 2] = normal.getZ();
	}
	return mesh;
    }
    
    shapes::Mesh* createMeshFromVertices(const std::vector<tf::Vector3> &source)
    {
	if (source.size() < 3)
	    return NULL;
	
	std::set<detail::myVertex, detail::ltVertexValue> vertices;
	std::vector<unsigned int>                         triangles;
	
	for (unsigned int i = 0 ; i < source.size() / 3 ; ++i)
	{
	    // check if we have new vertices
	    detail::myVertex vt;
	    
	    vt.point = source[3 * i];
	    std::set<detail::myVertex, detail::ltVertexValue>::iterator p1 = vertices.find(vt);
	    if (p1 == vertices.end())
	    {
		vt.index = vertices.size();
		vertices.insert(vt);		    
	    }
	    else
		vt.index = p1->index;
	    triangles.push_back(vt.index);		
	    
	    vt.point = source[3 * i + 1];
	    std::set<detail::myVertex, detail::ltVertexValue>::iterator p2 = vertices.find(vt);
	    if (p2 == vertices.end())
	    {
		vt.index = vertices.size();
		vertices.insert(vt);		    
	    }
	    else
		vt.index = p2->index;
	    triangles.push_back(vt.index);		
	    
	    vt.point = source[3 * i + 2];
	    std::set<detail::myVertex, detail::ltVertexValue>::iterator p3 = vertices.find(vt);
	    if (p3 == vertices.end())
	    {
		vt.index = vertices.size();
		vertices.insert(vt);		    
	    }
	    else
		vt.index = p3->index;

	    triangles.push_back(vt.index);
	}
	
	// sort our vertices
	std::vector<detail::myVertex> vt;
	vt.insert(vt.begin(), vertices.begin(), vertices.end());
	std::sort(vt.begin(), vt.end(), detail::ltVertexIndex());
	
	// copy the data to a mesh structure 
	unsigned int nt = triangles.size() / 3;
	
	shapes::Mesh *mesh = new shapes::Mesh(vt.size(), nt);
	for (unsigned int i = 0 ; i < vt.size() ; ++i)
	{
	    mesh->vertices[3 * i    ] = vt[i].point.getX();
	    mesh->vertices[3 * i + 1] = vt[i].point.getY();
	    mesh->vertices[3 * i + 2] = vt[i].point.getZ();
	}
	
	std::copy(triangles.begin(), triangles.end(), mesh->triangles);
	
	// compute normals 
	for (unsigned int i = 0 ; i < nt ; ++i)
	{
	    tf::Vector3 s1 = vt[triangles[i * 3    ]].point - vt[triangles[i * 3 + 1]].point;
	    tf::Vector3 s2 = vt[triangles[i * 3 + 1]].point - vt[triangles[i * 3 + 2]].point;
	    tf::Vector3 normal = s1.cross(s2);
	    normal.normalize();
	    mesh->normals[3 * i    ] = normal.getX();
	    mesh->normals[3 * i + 1] = normal.getY();
	    mesh->normals[3 * i + 2] = normal.getZ();
	}
	
	return mesh;
    }

    shapes::Mesh* createMeshFromBinaryStlData(const char *data, unsigned int size)
    {
	const char* pos = data;
	pos += 80; // skip the 80 byte header
	
	unsigned int numTriangles = *(unsigned int*)pos;
	pos += 4;
	
	// make sure we have read enough data
	if ((long)(50 * numTriangles + 84) <= size)
	{
	    std::vector<tf::Vector3> vertices;
	    
	    for (unsigned int currentTriangle = 0 ; currentTriangle < numTriangles ; ++currentTriangle)
	    {
		// skip the normal
		pos += 12;
		
		// read vertices 
		tf::Vector3 v1(0,0,0);
		tf::Vector3 v2(0,0,0);
		tf::Vector3 v3(0,0,0);
		
		v1.setX(*(float*)pos);
		pos += 4;
		v1.setY(*(float*)pos);
		pos += 4;
		v1.setZ(*(float*)pos);
		pos += 4;
		
		v2.setX(*(float*)pos);
		pos += 4;
		v2.setY(*(float*)pos);
		pos += 4;
		v2.setZ(*(float*)pos);
		pos += 4;
		
		v3.setX(*(float*)pos);
		pos += 4;
		v3.setY(*(float*)pos);
		pos += 4;
		v3.setZ(*(float*)pos);
		pos += 4;
		
		// skip attribute
		pos += 2;
		
		vertices.push_back(v1);
		vertices.push_back(v2);
		vertices.push_back(v3);
	    }
	    
	    return createMeshFromVertices(vertices);
	}
	
	return NULL;
    }
    
    shapes::Mesh* createMeshFromBinaryStl(const char *filename)
    {
	FILE* input = fopen(filename, "r");
	if (!input)
	    return NULL;
	
	fseek(input, 0, SEEK_END);
	long fileSize = ftell(input);
	fseek(input, 0, SEEK_SET);
	
	char* buffer = new char[fileSize];
	size_t rd = fread(buffer, fileSize, 1, input);
	
	fclose(input);
	
	shapes::Mesh *result = NULL;
	
	if (rd == 1)
    	    result = createMeshFromBinaryStlData(buffer, fileSize);
	
	delete[] buffer;
	
	return result;
    }
    
}
