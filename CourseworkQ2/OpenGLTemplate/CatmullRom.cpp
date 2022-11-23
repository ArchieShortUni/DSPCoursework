#include "CatmullRom.h"
#define _USE_MATH_DEFINES
#include <math.h>

struct Vertex {
	glm::vec3 pos;
	glm::vec3 normal;
	glm::vec2 tex;

	Vertex() {}

	Vertex(const glm::vec3& pos_in, const glm::vec2& tex_coord_in, const glm::vec3& normal_in)
	{
		pos = pos_in;
		normal = normal_in;
		tex = tex_coord_in;
	}
};

CCatmullRom::CCatmullRom()
{
	m_vertexCount = 0;

	m_texture.Load("resources\\textures\\Carpet\\carpet_texture.jpg");
	m_texture.SetSamplerObjectParameter(GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	m_texture.SetSamplerObjectParameter(GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	m_texture.SetSamplerObjectParameter(GL_TEXTURE_WRAP_S, GL_REPEAT);
	m_texture.SetSamplerObjectParameter(GL_TEXTURE_WRAP_T, GL_REPEAT);
}

CCatmullRom::~CCatmullRom()
{}

// Perform Catmull Rom spline interpolation between four points, interpolating the space between p1 and p2
glm::vec3 CCatmullRom::Interpolate(glm::vec3 &p0, glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;

	glm::vec3 a = p1;
	glm::vec3 b = 0.5f * (-p0 + p2);
	glm::vec3 c = 0.5f * (2.0f*p0 - 5.0f*p1 + 4.0f*p2 - p3);
	glm::vec3 d = 0.5f * (-p0 + 3.0f*p1 - 3.0f*p2 + p3);

	return a + b*t + c*t2 + d*t3;

}


void CCatmullRom::SetControlPoints()
{
	// Set control points (m_controlPoints) here, or load from disk
	m_controlPoints.push_back(glm::vec3(0, 5, 0));	//0
	m_controlPoints.push_back(glm::vec3(-60, 5, 100));	//1
	m_controlPoints.push_back(glm::vec3(0, 5, 200));	//2
	m_controlPoints.push_back(glm::vec3(60, 5, 100));	//3

	m_controlPoints.push_back(glm::vec3(0, 5, 0));	//4
	m_controlPoints.push_back(glm::vec3(-60, 5, -100));	//5
	m_controlPoints.push_back(glm::vec3(0, 5, -200));	//6
	m_controlPoints.push_back(glm::vec3(60, 5, -100));	//7




	// Optionally, set upvectors (m_controlUpVectors, one for each control point as well)
	m_controlUpVectors.push_back(glm::vec3(0,1,0));
	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));
	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));
	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));

	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));
	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));
	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));
	m_controlUpVectors.push_back(glm::vec3(0, 1, 0));




}


// Determine lengths along the control points, which is the set of control points forming the closed curve
void CCatmullRom::ComputeLengthsAlongControlPoints()
{
	int M = (int) m_controlPoints.size();

	float fAccumulatedLength = 0.0f;
	m_distances.push_back(fAccumulatedLength);
	for (int i = 1; i < M; i++) {
		fAccumulatedLength += glm::distance(m_controlPoints[i-1], m_controlPoints[i]);
		m_distances.push_back(fAccumulatedLength);
	}

	// Get the distance from the last point to the first
	fAccumulatedLength += glm::distance(m_controlPoints[M-1], m_controlPoints[0]);
	m_distances.push_back(fAccumulatedLength);
}


// Return the point (and upvector, if control upvectors provided) based on a distance d along the control polygon
bool CCatmullRom::Sample(float d, glm::vec3 &p, glm::vec3 &up)
{
	if (d < 0)
		return false;

	int M = (int) m_controlPoints.size();
	if (M == 0)
		return false;


	 fTotalLength = m_distances[m_distances.size() - 1];

	// The the current length along the control polygon; handle the case where we've looped around the track
	float fLength = d - (int) (d / fTotalLength) * fTotalLength;

	// Find the current segment
	int j = -1;
	for (int i = 0; i < (int)m_distances.size()-1; i++) {
		if (fLength >= m_distances[i] && fLength < m_distances[i + 1]) {
			j = i; // found it!
			break;
		}
	}

	if (j == -1)
		return false;

	// Interpolate on current segment -- get t
	float fSegmentLength = m_distances[j + 1] - m_distances[j];
	float t = (fLength - m_distances[j]) / fSegmentLength;
	
	// Get the indices of the four points along the control polygon for the current segment
	int iPrev = ((j-1) + M) % M;
	int iCur = j;
	int iNext = (j + 1) % M;
	int iNextNext = (j + 2) % M;

	// Interpolate to get the point (and upvector)
	p = Interpolate(m_controlPoints[iPrev], m_controlPoints[iCur], m_controlPoints[iNext], m_controlPoints[iNextNext], t);
	if (m_controlUpVectors.size() == m_controlPoints.size())
		up = glm::normalize(Interpolate(m_controlUpVectors[iPrev], m_controlUpVectors[iCur], m_controlUpVectors[iNext], m_controlUpVectors[iNextNext], t));

	return true;
}



// Sample a set of control points using an open Catmull-Rom spline, to produce a set of iNumSamples that are (roughly) equally spaced
void CCatmullRom::UniformlySampleControlPoints(int numSamples)
{
	glm::vec3 p, up;

	// Compute the lengths of each segment along the control polygon, and the total length
	ComputeLengthsAlongControlPoints();
	float fTotalLength = m_distances[m_distances.size() - 1];

	// The spacing will be based on the control polygon
	float fSpacing = fTotalLength / numSamples;

	// Call PointAt to sample the spline, to generate the points
	for (int i = 0; i < numSamples; i++) {
		Sample(i * fSpacing, p, up);
		m_centrelinePoints.push_back(p);
		if (m_controlUpVectors.size() > 0)
			m_centrelineUpVectors.push_back(up);

	}


	// Repeat once more for truly equidistant points
	m_controlPoints = m_centrelinePoints;
	m_controlUpVectors = m_centrelineUpVectors;
	m_centrelinePoints.clear();
	m_centrelineUpVectors.clear();
	m_distances.clear();
	ComputeLengthsAlongControlPoints();
	fTotalLength = m_distances[m_distances.size() - 1];
	fSpacing = fTotalLength / numSamples;
	for (int i = 0; i < numSamples; i++) {
		Sample(i * fSpacing, p, up);
		m_centrelinePoints.push_back(p);
		if (m_controlUpVectors.size() > 0)
			m_centrelineUpVectors.push_back(up);
	}


}



void CCatmullRom::CreateCentreline()
{
	// Call Set Control Points
	SetControlPoints(); 
	// Call UniformlySampleControlPoints with the number of samples required
	UniformlySampleControlPoints(2000);

	// Create a VAO called m_vaoCentreline and a VBO to get the points onto the graphics card

	glGenVertexArrays(1, &m_vaoCentreline);
	glBindVertexArray(m_vaoCentreline);
	// Create a VBO
	CVertexBufferObject vbo;
	vbo.Create();
	vbo.Bind();
	glm::vec2 texCoord(0.0f, 0.0f);
	glm::vec3 normal(0.0f, 1.0f, 0.0f);
	for (unsigned int i = 0; i < m_centrelinePoints.size(); i++) {
		
		vbo.AddData(&m_centrelinePoints.at(i), sizeof(glm::vec3));
		vbo.AddData(&texCoord, sizeof(glm::vec2));
		vbo.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	GLsizei stride = 2 * sizeof(glm::vec3) + sizeof(glm::vec2);
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));
}

void CCatmullRom::SinHeightGenerator(float t, glm::vec3& current_position) {
	float newHeight = 0.f;
	float pi = 2 * acos(0.0);
	newHeight = amplitude * sin(2 * pi * t * frequency);
	
	current_position += glm::vec3(0.f, newHeight, 0.f);
	
}

void CCatmullRom::CreateOffsetCurves()
{
	float interval = 0.125;
	// Compute the offset curves, one left, and one right.  Store the points in m_leftOffsetPoints and m_rightOffsetPoints respectively
	for (int i = 0; i < m_centrelinePoints.size()-1 ; i++) {
		glm::vec3 currentPoint = m_centrelinePoints.at(i);
		glm::vec3 nextPoint = m_centrelinePoints.at(i+1);
		glm::vec3 normal =glm::cross( glm::normalize(nextPoint - currentPoint) ,m_centrelineUpVectors.at(i));


		glm::vec3 left4 = currentPoint - (width * (interval * 4)) * normal;
		glm::vec3 left3 = currentPoint - (width * (interval * 3)) * normal;
		glm::vec3 left2 = currentPoint - (width * (interval * 2)) * normal;
		glm::vec3 left1 = currentPoint - (width * (interval * 1)) * normal;

		glm::vec3 right4 = currentPoint + (width * (interval * 4)) * normal;
		glm::vec3 right3 = currentPoint + (width * (interval * 3)) * normal;
		glm::vec3 right2 = currentPoint + (width * (interval * 2)) * normal;
		glm::vec3 right1 = currentPoint + (width * (interval * 1)) * normal;

		m_leftOffsetPoints1.push_back(left1);
		m_leftOffsetPoints2.push_back(left2);
		m_leftOffsetPoints3.push_back(left3);
		m_leftOffsetPoints4.push_back(left4);

		m_rightOffsetPoints1.push_back(right1);
		m_rightOffsetPoints2.push_back(right2);
		m_rightOffsetPoints3.push_back(right3);
		m_rightOffsetPoints4.push_back(right4);

	}
	// Generate two VAOs called m_vaoLeftOffsetCurve and m_vaoRightOffsetCurve, each with a VBO, and get the offset curve points on the graphics card
	// Note it is possible to only use one VAO / VBO with all the points instead.

	glm::vec2 texCoord(0.0f, 0.0f);
	glm::vec3 normal(0.0f, 1.0f, 0.0f);
	GLsizei stride = 2 * sizeof(glm::vec3) + sizeof(glm::vec2);


	glGenVertexArrays(1, &m_vaoLeftTrack1);
	glBindVertexArray(m_vaoLeftTrack1);
	// Create a VBO
	CVertexBufferObject vbo;
	vbo.Create();
	vbo.Bind();
	for (unsigned int i = 0; i < m_leftOffsetPoints1.size(); i++) {

		vbo.AddData(&m_leftOffsetPoints1.at(i), sizeof(glm::vec3));
		vbo.AddData(&texCoord, sizeof(glm::vec2));
		vbo.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));

	//Offset Left 2 
	glGenVertexArrays(1, &m_vaoLeftTrack2);
	glBindVertexArray(m_vaoLeftTrack2);
	// Create a VBO
	CVertexBufferObject vbo2;
	vbo2.Create();
	vbo2.Bind();
	for (unsigned int i = 0; i < m_leftOffsetPoints2.size(); i++) {

		vbo2.AddData(&m_leftOffsetPoints2.at(i), sizeof(glm::vec3));
		vbo2.AddData(&texCoord, sizeof(glm::vec2));
		vbo2.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo2.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));


	//OFFSET LEFT 3
	glGenVertexArrays(1, &m_vaoLeftTrack3);
	glBindVertexArray(m_vaoLeftTrack3);
	// Create a VBO
	CVertexBufferObject vbo3;
	vbo3.Create();
	vbo3.Bind();
	for (unsigned int i = 0; i < m_leftOffsetPoints3.size(); i++) {

		vbo3.AddData(&m_leftOffsetPoints3.at(i), sizeof(glm::vec3));
		vbo3.AddData(&texCoord, sizeof(glm::vec2));
		vbo3.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo3.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));


	//OFFSET LEFT 4
	glGenVertexArrays(1, &m_vaoLeftTrack4);
	glBindVertexArray(m_vaoLeftTrack4);
	// Create a VBO
	CVertexBufferObject vbo4;
	vbo4.Create();
	vbo4.Bind();
	for (unsigned int i = 0; i < m_leftOffsetPoints4.size(); i++) {

		vbo4.AddData(&m_leftOffsetPoints4.at(i), sizeof(glm::vec3));
		vbo4.AddData(&texCoord, sizeof(glm::vec2));
		vbo4.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo4.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));



	//RIGHT TRACK
	//OFFSET RIGHT 1

	glGenVertexArrays(1, &m_vaoRightTrack1);
	glBindVertexArray(m_vaoRightTrack1);
	// Create a VBO
	CVertexBufferObject vbo5;
	vbo5.Create();
	vbo5.Bind();
	for (unsigned int i = 0; i < m_rightOffsetPoints1.size(); i++) {

		vbo5.AddData(&m_rightOffsetPoints1.at(i), sizeof(glm::vec3));
		vbo5.AddData(&texCoord, sizeof(glm::vec2));
		vbo5.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo5.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));

	
	//OFFSET RIGHT 2 
	glGenVertexArrays(1, &m_vaoRightTrack2);
	glBindVertexArray(m_vaoRightTrack2);
	// Create a VBO
	CVertexBufferObject vbo6;
	vbo6.Create();
	vbo6.Bind();
	for (unsigned int i = 0; i < m_rightOffsetPoints2.size(); i++) {

		vbo6.AddData(&m_rightOffsetPoints2.at(i), sizeof(glm::vec3));
		vbo6.AddData(&texCoord, sizeof(glm::vec2));
		vbo6.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo6.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));


	//OFFSET RIGHT 3
	glGenVertexArrays(1, &m_vaoRightTrack3);
	glBindVertexArray(m_vaoRightTrack3);
	// Create a VBO
	CVertexBufferObject vbo7;
	vbo7.Create();
	vbo7.Bind();
	for (unsigned int i = 0; i < m_rightOffsetPoints3.size(); i++) {

		vbo7.AddData(&m_rightOffsetPoints3.at(i), sizeof(glm::vec3));
		vbo7.AddData(&texCoord, sizeof(glm::vec2));
		vbo7.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo7.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));


	//OFFSET RIGHT 4
	glGenVertexArrays(1, &m_vaoRightTrack4);
	glBindVertexArray(m_vaoRightTrack4);
	// Create a VBO
	CVertexBufferObject vbo8;
	vbo8.Create();
	vbo8.Bind();
	for (unsigned int i = 0; i < m_rightOffsetPoints4.size(); i++) {

		vbo8.AddData(&m_rightOffsetPoints4.at(i), sizeof(glm::vec3));
		vbo8.AddData(&texCoord, sizeof(glm::vec2));
		vbo8.AddData(&normal, sizeof(glm::vec3));
	}
	// Upload the VBO to the GPU
	vbo8.UploadDataToGPU(GL_STATIC_DRAW);
	// Set the vertex attribute locations
	// Vertex positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, 0);
	// Texture coordinates
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::vec3));
	// Normal vectors
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
		(void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));


}


void CCatmullRom::CreateTrack()
{
	glGenVertexArrays(1, &m_vaoTrack);
	glBindVertexArray(m_vaoTrack);
	// Generate a VAO called m_vaoTrack and a VBO to get the offset curve points and indices on the graphics card

	
	glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec2 t00 = glm::vec2(.15f, 0.0f); //Bottom Left
	glm::vec2 t01 = glm::vec2(0.15f, .08f); //Top Left
	glm::vec2 t10 = glm::vec2(.2375f, 0.0f); //Bottom Right
	glm::vec2 t11 = glm::vec2(.2375f, .08f); //Top Right

	
	CVertexBufferObject vboTrack;

	vboTrack.Create();
	vboTrack.Bind();
	//for (unsigned int i = 0; i < m_rightOffsetPoints.size(); i++) {
	glm::vec2 wrappingConstantY = glm::vec2(0.f,0.08);
	glm::vec2 wrappingConstantX = glm::vec2(.0875f, 0.0f);

		for (int i = 0; i < m_rightOffsetPoints4.size()-1; i += 1) {

			//Section 1 of 8
			vboTrack.AddData(&m_rightOffsetPoints4.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints4.at(i+1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints3.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_rightOffsetPoints3.at(i+1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			//Section 2 of 8
			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			vboTrack.AddData(&m_rightOffsetPoints3.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints3.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints2.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_rightOffsetPoints2.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			//Section 3 of 8
			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			vboTrack.AddData(&m_rightOffsetPoints2.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints2.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints1.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_rightOffsetPoints1.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			//Section 4 of 8
			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			vboTrack.AddData(&m_rightOffsetPoints1.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_rightOffsetPoints1.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_centrelinePoints.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_centrelinePoints.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			//Section 5 of 8
			vboTrack.AddData(&m_centrelinePoints.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_centrelinePoints.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints1.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_leftOffsetPoints1.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			//Section 6 of 8
			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			vboTrack.AddData(&m_leftOffsetPoints1.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints1.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints2.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_leftOffsetPoints2.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			//Section 7 of 8
			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			vboTrack.AddData(&m_leftOffsetPoints2.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints2.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints3.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_leftOffsetPoints3.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			//Section 8 of 8
			t00 += wrappingConstantX;
			t01 += wrappingConstantX;
			t10 += wrappingConstantX;
			t11 += wrappingConstantX;

			vboTrack.AddData(&m_leftOffsetPoints3.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t00, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints3.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t01, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			vboTrack.AddData(&m_leftOffsetPoints4.at(i), sizeof(glm::vec3));
			vboTrack.AddData(&t10, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));


			vboTrack.AddData(&m_leftOffsetPoints4.at(i + 1), sizeof(glm::vec3));
			vboTrack.AddData(&t11, sizeof(glm::vec2));
			vboTrack.AddData(&normal, sizeof(glm::vec3));

			t00 = glm::vec2(0.15f,t00.y);
			t01 = glm::vec2(0.15f, t01.y);
			t10 = glm::vec2(.2375f, t10.y);
			t11 = glm::vec2(.2375f, t11.y);


			t00 += wrappingConstantY;
			t01 += wrappingConstantY;
			t10 += wrappingConstantY;
			t11 += wrappingConstantY;
		}

	//FINAL LINK	
	//Section 1 of 8
		t01 += glm::vec2(0,0.08);
		t11 += glm::vec2(0, 0.08);

		vboTrack.AddData(&m_rightOffsetPoints4.at(m_rightOffsetPoints4.size()-1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints4.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints3.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_rightOffsetPoints3.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		//Section 2 of 8
		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		vboTrack.AddData(&m_rightOffsetPoints3.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints3.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints2.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_rightOffsetPoints2.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		//Section 3 of 8
		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		vboTrack.AddData(&m_rightOffsetPoints2.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints2.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints1.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_rightOffsetPoints1.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		//Section 4 of 8
		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		vboTrack.AddData(&m_rightOffsetPoints1.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_rightOffsetPoints1.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_centrelinePoints.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_centrelinePoints.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		//Section 5 of 8
		vboTrack.AddData(&m_centrelinePoints.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_centrelinePoints.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints1.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_leftOffsetPoints1.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		//Section 6 of 8
		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		vboTrack.AddData(&m_leftOffsetPoints1.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints1.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints2.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_leftOffsetPoints2.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		//Section 7 of 8
		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		vboTrack.AddData(&m_leftOffsetPoints2.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints2.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints3.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_leftOffsetPoints3.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		//Section 8 of 8
		t00 += wrappingConstantX;
		t01 += wrappingConstantX;
		t10 += wrappingConstantX;
		t11 += wrappingConstantX;

		vboTrack.AddData(&m_leftOffsetPoints3.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t00, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints3.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t01, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

		vboTrack.AddData(&m_leftOffsetPoints4.at(m_rightOffsetPoints4.size() - 1), sizeof(glm::vec3));
		vboTrack.AddData(&t10, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));


		vboTrack.AddData(&m_leftOffsetPoints4.at(0), sizeof(glm::vec3));
		vboTrack.AddData(&t11, sizeof(glm::vec2));
		vboTrack.AddData(&normal, sizeof(glm::vec3));

	
	
	// Upload the VBO to the GPU
	vboTrack.UploadDataToGPU(GL_STATIC_DRAW);

	
	//positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);

	//textures
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(3 * sizeof(float)));

	//normals
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(sizeof(glm::vec3) + sizeof(glm::vec2)));


}


void CCatmullRom::RenderCentreline()
{
	glLineWidth(3.f);
	glPointSize(7.f);
	glBindVertexArray(m_vaoCentreline);
	glDrawArrays(GL_LINE_STRIP, 0, m_centrelinePoints.size());

	glDrawArrays(GL_POINTS, 0, m_centrelinePoints.size());



	
	}


void CCatmullRom::RenderOffsetCurves()
{
	glLineWidth(3.f);
	glPointSize(7.f);
	glBindVertexArray(m_vaoLeftTrack1);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());
	//// Bind the VAO m_vaoCentreline and render it

	glBindVertexArray(m_vaoLeftTrack2);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());

	glBindVertexArray(m_vaoLeftTrack3);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());

	glBindVertexArray(m_vaoLeftTrack4);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());

	glBindVertexArray(m_vaoRightTrack1);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());
	//// Bind the VAO m_vaoCentreline and render it

	glBindVertexArray(m_vaoRightTrack2);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());

	glBindVertexArray(m_vaoRightTrack3);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());

	glBindVertexArray(m_vaoRightTrack4);
	glDrawArrays(GL_LINE_STRIP, 0, m_leftOffsetPoints1.size());
	glDrawArrays(GL_POINTS, 0, m_leftOffsetPoints1.size());

	// Bind the VAO m_vaoLeftOffsetCurve and render it

	// Bind the VAO m_vaoRightOffsetCurve and render it
}


void CCatmullRom::RenderTrack()
{
	// Bind the VAO m_vaoTrack and render it
	//Textured Top
	glBindVertexArray(m_vaoTrack);
	m_texture.Bind();
	// Call glDrawArrays to render each side
	for (int i = 0; i <( m_centrelinePoints.size()*32)+4; i += 4) {
		glDrawArrays(GL_TRIANGLE_STRIP, i, 4);

	}
}

int CCatmullRom::CurrentLap(float d)
{

	return (int)(d / m_distances.back());

}

glm::vec3 CCatmullRom::_dummy_vector(0.0f, 0.0f, 0.0f);