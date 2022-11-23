#pragma once
#include "Common.h"
#include "vertexBufferObject.h"
#include "vertexBufferObjectIndexed.h"
#include "Texture.h"



class CCatmullRom
{
public:
	CCatmullRom();
	~CCatmullRom();

	void CreateCentreline();
	void RenderCentreline();

	void CreateOffsetCurves();
	void RenderOffsetCurves();

	void CreateTrack();
	void RenderTrack();

	void SinHeightGenerator(float t, glm::vec3& current_position);

	int CurrentLap(float d); // Return the currvent lap (starting from 0) based on distance along the control curve.

	bool Sample(float d, glm::vec3 &p, glm::vec3 &up = _dummy_vector); // Return a point on the centreline based on a certain distance along the control curve.

	float get_total_length() { return fTotalLength; }
private:

	//Sin wave Variables
	float frequency =400;
	float amplitude = 1.f;

	
	float fTotalLength = 0;
	void SetControlPoints();
	void ComputeLengthsAlongControlPoints();
	void UniformlySampleControlPoints(int numSamples);
	glm::vec3 Interpolate(glm::vec3 &p0, glm::vec3 &p1, glm::vec3 &p2, glm::vec3 &p3, float t);

	float width = 10;

	vector<float> m_distances;
	CTexture m_texture;

	
	GLuint m_vbo; 
	GLuint m_vaoCentreline;
	GLuint m_vaoLeftOffsetCurve;
	GLuint m_vaoRightOffsetCurve;


	//Carpet Tracks VAO
	GLuint m_vaoLeftTrack1;
	GLuint m_vaoLeftTrack2;
	GLuint m_vaoLeftTrack3;
	GLuint m_vaoLeftTrack4;


	GLuint m_vaoRightTrack1;
	GLuint m_vaoRightTrack2;
	GLuint m_vaoRightTrack3;
	GLuint m_vaoRightTrack4;



	GLuint m_vboTrack;
	GLuint m_vaoTrack;

	static glm::vec3 _dummy_vector;
	vector<glm::vec3> m_controlPoints;		// Control points, which are interpolated to produce the centreline points
	vector<glm::vec3> m_controlUpVectors;	// Control upvectors, which are interpolated to produce the centreline upvectors
	vector<glm::vec3> m_centrelinePoints;	// Centreline points
	vector<glm::vec3> m_centrelineUpVectors;// Centreline upvectors

	vector<glm::vec3> m_leftOffsetPoints;	// Left offset curve points
	vector<glm::vec3> m_rightOffsetPoints;	// Right offset curve points

	//Carpet Tracks Vectors 4 outside 1 closest to the middle
	vector<glm::vec3> m_leftOffsetPoints1;	
	vector<glm::vec3> m_leftOffsetPoints2;
	vector<glm::vec3> m_leftOffsetPoints3;
	vector<glm::vec3> m_leftOffsetPoints4;

	vector<glm::vec3> m_rightOffsetPoints1;
	vector<glm::vec3> m_rightOffsetPoints2;
	vector<glm::vec3> m_rightOffsetPoints3;
	vector<glm::vec3> m_rightOffsetPoints4;



	unsigned int m_vertexCount;				// Number of vertices in the track VBO
};
