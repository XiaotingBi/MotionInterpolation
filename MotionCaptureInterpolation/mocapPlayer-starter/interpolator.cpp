#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
	/*   [cos a2  -sin a2  0] [cos a1   0  sin a1] [1    0           0]

	R =  [sin a2   cos a2  0] [0        1       0] [0  cos a0  -sin a0]

	     [0           0    1] [-sin a1  0  cos a1] [0  sin a0   cos a0]
	*/
	double temp_angle[3];
	for(int i=0; i<3; i++)
		temp_angle[i] = angles[i] / 180 * M_PI;

	double t00 = cos(temp_angle[2])*cos(temp_angle[1]);
	double t01 = -sin(temp_angle[2]);
	double t02 = sin(temp_angle[1])*cos(temp_angle[2]);
	double t10 = sin(temp_angle[2])*cos(temp_angle[1]);
	double t11 = cos(temp_angle[2]);
	double t12 = sin(temp_angle[1])*sin(temp_angle[2]);
	double t20 = -sin(temp_angle[1]);
	double t21 = 0;
	double t22 = cos(temp_angle[1]);

	R[0] = t00*1;
	R[1] = t01*cos(temp_angle[0]) + t02*sin(temp_angle[0]);
	R[2] = t01*(-1)*sin(temp_angle[0]) + t02*cos(temp_angle[0]);
	R[3] = t10;
	R[4] = t11*cos(temp_angle[0]) + t12*sin(temp_angle[0]);
	R[5] = t11*(-1)*sin(temp_angle[0]) + t12*cos(temp_angle[0]);
	R[6] = t20;
	R[7] = t21*cos(temp_angle[0]) + t22*sin(temp_angle[0]);
	R[8] = t21*(-1)*sin(temp_angle[0]) + t22*cos(temp_angle[0]);
	
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// points: q(n-1), q(n), q(n+1), q(n+2)
	vector q_n_1, q_n, q_n1, q_n2;
	vector a_n, b_n1;
	vector slerp_a, a_n_, slerp_b, b_n_;

	// bone rotation
	vector b_rotate_n_1, b_rotate_n, b_rotate_n1, b_rotate_n2;
	vector b_a_n, b_b_n1;
	vector b_slerp_a, b_a_n_, b_slerp_b, b_b_n_;

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;
		int pre_keyframe = startKeyframe - N - 1;
		int next_keyframe = endKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		Posture * pre_posture;
		Posture * next_posture;

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);
		// previous posture
		if (pre_keyframe >= 0)
			pre_posture = pInputMotion->GetPosture(pre_keyframe);
		else
			pre_posture = NULL;
		// next posture
		if (next_keyframe < inputLength)
		{
			next_posture = pInputMotion->GetPosture(next_keyframe);
			pOutputMotion->SetPosture(next_keyframe, *next_posture);
		}
		else
			next_posture = NULL;

		//printf("1**********\n");

		// interpolate in between
		for(int frame=1; frame<=N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N+1);

			// interpolate root position
			q_n = startPosture->root_pos;
			q_n1 = endPosture->root_pos;

			//printf("2**********\n");

			if (startKeyframe > 0 && next_keyframe <= inputLength)
			{
				//printf("2**********\n");
				q_n_1 = pre_posture->root_pos;
				q_n2 = next_posture->root_pos;
						
				slerp_a = q_n + q_n - q_n_1; // temp - q(n) = q(n) - q(n-1)
				a_n_ = 0.5 * (q_n1 + slerp_a); // a_(n) = 1/2 ( q(n+1) + temp )
				a_n = (a_n_ - q_n) * (1.0/3.0) + q_n; // a(n) = 1/3 (q(n), a_(n))
				// must be like this: (1.0/3.0)!!! Or it will be problem!!!
				slerp_b = q_n1 + q_n1 - q_n;
				b_n_ = 0.5 * (q_n2 + slerp_b);
				b_n1 = (b_n_ - q_n1) * (1.0/3.0) + q_n1;
				b_n1 = q_n1 - b_n1 + q_n1;
			} else if (startKeyframe == 0)
			{
				//printf("3**********\n");
				// the first frame doesn't have previous frame
				//if (!next_posture) printf("Null\n");
				q_n2 = next_posture->root_pos;
				slerp_a = q_n1 + q_n1 - q_n2;
				a_n = (slerp_a - q_n) * (1.0/3.0) + q_n;
				
				slerp_b = q_n1 - q_n + q_n1;
				b_n_ = (q_n2 + slerp_b) * 0.5;
				b_n1 = (b_n_ - q_n1) * (1.0/3.0) + q_n1;
				b_n1 = q_n1 - b_n1 + q_n1;
			} else
			{
				//printf("4**********\n");
				// next_keyframe > inputLength
				// doesn't have next frame
				q_n_1 = pre_posture->root_pos;
				slerp_a = q_n + q_n - q_n_1;
				a_n_ = (slerp_a + q_n1) * 0.5;
				a_n = (a_n_ - q_n) * (1.0/3.0) + q_n;

				slerp_b = q_n + q_n - q_n_1;
				b_n1 = (slerp_b - q_n1) * (1.0/3.0) + q_n1;
			}
			//printf("4.5\n");
			interpolatedPosture.root_pos = DeCasteljauEuler(t, q_n, a_n, b_n1, q_n1);
			//printf("5**********\n");
			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				b_rotate_n = startPosture->bone_rotation[bone];
				b_rotate_n1 = endPosture->bone_rotation[bone];


				if (startKeyframe > 0 && next_keyframe <= inputLength)
				{
					b_rotate_n_1 = pre_posture->bone_rotation[bone];
					b_rotate_n2 = next_posture->bone_rotation[bone];

					b_slerp_a = b_rotate_n + b_rotate_n - b_rotate_n_1; 
					b_a_n_ = 0.5 * (b_rotate_n1 + b_slerp_a); 
					b_a_n = (b_a_n_ - b_rotate_n) * (1.0/3.0) + b_rotate_n; 

					b_slerp_b = b_rotate_n1 + b_rotate_n1 - b_rotate_n;
					b_b_n_ = 0.5 * (b_rotate_n2 + b_slerp_b);
					b_b_n1 = (b_b_n_ - b_rotate_n1) * (1.0/3.0) + b_rotate_n1;
					b_b_n1 = b_rotate_n1 - b_b_n1 + b_rotate_n1;
				} 
				else if (startKeyframe <= 0)
				{
					// the first frame doesn't have previous frame
					b_rotate_n2 = next_posture->bone_rotation[bone];
					b_slerp_a = b_rotate_n1 + b_rotate_n1 - b_rotate_n2;
					b_a_n = (b_slerp_a - b_rotate_n) * (1.0/3.0) + b_rotate_n;

					b_slerp_b = b_rotate_n1 - b_rotate_n + b_rotate_n1;
					b_b_n_ = 0.5 * (b_rotate_n2 + b_slerp_b);
					b_b_n1 = (b_b_n_ - b_rotate_n1) * (1.0/3.0) + b_rotate_n1;
					b_b_n1 = b_rotate_n1 - b_b_n1 + b_rotate_n1;
				}
				else
				{
					// next_keyframe > inputLength
					// doesn't have next frame
					b_rotate_n_1 = pre_posture->bone_rotation[bone];
					b_slerp_a = b_rotate_n + b_rotate_n - b_rotate_n_1;
					b_a_n_ = 0.5 * (b_slerp_a + b_rotate_n1);
					b_a_n = (b_a_n_ - b_rotate_n) * (1.0/3.0) + b_rotate_n;

					b_slerp_b = b_rotate_n + b_rotate_n - b_rotate_n_1;
					b_b_n1 = (b_slerp_b - b_rotate_n1) * (1.0/3.0) + b_rotate_n1;
				}
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, b_rotate_n, b_a_n, b_b_n1, b_rotate_n1);
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for(int frame=startKeyframe+1; frame<inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
	// Linear Interpolation Quaternion implementation
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;

			// initialize the start, end and interpolate orientation on every bone in the form of quaternion
			Quaternion<double> bStart, bEnd, inter;
			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				// convert from euler angles to quaternion
				Euler2Quaternion(startPosture->bone_rotation[bone].p, bStart);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, bEnd);
				// interpolate bone orientation in the form of quaternion
				inter = Slerp(t, bStart, bEnd);
				// convert from quaternion back to euler angles
				Quaternion2Euler(inter, interpolatedPosture.bone_rotation[bone].p);
			}
			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}
		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
	{
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
	}
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
	//modify BezierInterpolationEuler function
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// points: q(n-1), q(n), q(n+1), q(n+2)
	vector q_n_1, q_n, q_n1, q_n2;
	// bone rotation
	Quaternion<double> b_rotate_n_1, b_rotate_n, b_rotate_n1, b_rotate_n2;

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength)
	{
		int endKeyframe = startKeyframe + N + 1;
		int pre_keyframe = startKeyframe - N - 1;
		int next_keyframe = endKeyframe + N + 1;

		Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
		Posture * pre_posture;
		Posture * next_posture;

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);
		// previous posture
		if (pre_keyframe >= 0)
			pre_posture = pInputMotion->GetPosture(pre_keyframe);
		else
			pre_posture = NULL;
		// next posture
		if (next_keyframe < inputLength)
		{
			next_posture = pInputMotion->GetPosture(next_keyframe);
			pOutputMotion->SetPosture(next_keyframe, *next_posture);
		}
		else
			next_posture = NULL;

		// interpolate in between
		for(int frame=1; frame<=N; frame++)
		{
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N+1);

			// interpolate root position
			q_n = startPosture->root_pos;
			q_n1 = endPosture->root_pos;

			vector slerp_a, a_n_, slerp_b, b_n_;
			vector a_n, b_n1;

			if (startKeyframe > 0 && next_keyframe <= inputLength)
			{
				q_n_1 = pre_posture->root_pos;
				q_n2 = next_posture->root_pos;

				slerp_a = q_n + q_n - q_n_1; // temp - q(n) = q(n) - q(n-1)
				a_n_ = 0.5 * (q_n1 + slerp_a); // a_(n) = 1/2 ( q(n+1) + temp )
				a_n = (a_n_ - q_n) * (1.0/3.0) + q_n; // a(n) = 1/3 (q(n), a_(n))

				slerp_b = q_n1 + q_n1 - q_n;
				b_n_ = 0.5 * (q_n2 + slerp_b);
				b_n1 = (b_n_ - q_n1) * (1.0/3.0) + q_n1;
				b_n1 = q_n1 - b_n1 + q_n1;
			} else if (startKeyframe <= 0)
			{
				// the first frame doesn't have previous frame
				q_n2 = next_posture->root_pos;
				slerp_a = q_n1 + q_n1 - q_n2;
				a_n = (slerp_a - q_n) * (1.0/3.0) + q_n;

				slerp_b = q_n1 - q_n + q_n1;
				b_n_ = 0.5 * (q_n2 + slerp_b);
				b_n1 = (b_n_ - q_n1) * (1.0/3.0) + q_n1;
				b_n1 = q_n1 - b_n1 + q_n1;
			} else
			{
				// next_keyframe > inputLength
				// doesn't have next frame
				q_n_1 = pre_posture->root_pos;
				slerp_a = q_n + q_n - q_n_1;
				a_n_ = 0.5 * (slerp_a + q_n1);
				a_n = (a_n_ - q_n) * (1.0/3.0) + q_n;

				slerp_b = q_n + q_n - q_n_1;
				b_n1 = (slerp_b - q_n1) * (1.0/3.0) + q_n1;
			}

			interpolatedPosture.root_pos = DeCasteljauEuler(t, q_n, a_n, b_n1, q_n1);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				// modify BezierInterpolationEuler function
				//b_rotate_n = startPosture->bone_rotation[bone];
				//b_rotate_n1 = endPosture->bone_rotation[bone];
				Euler2Quaternion(startPosture->bone_rotation[bone].p, b_rotate_n);
				Euler2Quaternion(endPosture->bone_rotation[bone].p, b_rotate_n1);

				Quaternion<double> b_a_n, b_b_n1;
				Quaternion<double> b_slerp_a, b_a_n_, b_slerp_b, b_b_n_;

				if (startKeyframe > 0 && next_keyframe <= inputLength)
				{
					//b_rotate_n_1 = pre_posture->bone_rotation[bone];
					//b_rotate_n2 = next_posture->bone_rotation[bone];
					Euler2Quaternion(pre_posture->bone_rotation[bone].p, b_rotate_n_1);
					Euler2Quaternion(next_posture->bone_rotation[bone].p, b_rotate_n2);

					//b_slerp_a = b_rotate_n + b_rotate_n - b_rotate_n_1;
					b_slerp_a = Double(b_rotate_n_1, b_rotate_n);
					//b_a_n_ = 1/2 * (b_rotate_n1 + b_slerp_a); 
					b_a_n_ = Slerp(0.5, b_slerp_a, b_rotate_n1);
					//b_a_n = (b_a_n_ - b_rotate_n) * 1/3 + b_rotate_n; 
					b_a_n = Slerp((1.0/3.0), b_rotate_n, b_a_n_);

					//b_slerp_b = b_rotate_n1 + b_rotate_n1 - b_rotate_n;
					b_slerp_b = Double(b_rotate_n, b_rotate_n1);
					//b_b_n_ = 1/2 * (b_rotate_n2 + b_slerp_b);
					b_b_n_ = Slerp(0.5, b_slerp_b, b_rotate_n2);
					//b_b_n1 = (b_b_n_ - b_rotate_n1) * 1/3 + b_rotate_n1;					
					//b_b_n1 = b_rotate_n1 - b_b_n1 + b_rotate_n1;
					b_b_n1 = Slerp(-(1.0/3.0), b_rotate_n1, b_b_n_);
				} 
				else if (startKeyframe <= 0)
				{
					// the first frame doesn't have previous frame
					//b_rotate_n2 = next_posture->bone_rotation[bone];
					Euler2Quaternion(next_posture->bone_rotation[bone].p, b_rotate_n2);
					//b_slerp_a = b_rotate_n1 + b_rotate_n1 - b_rotate_n2;
					b_slerp_a = Double(b_rotate_n2, b_rotate_n1);
					//b_a_n = (b_slerp_a - b_rotate_n) * 1/3 + b_rotate_n;
					b_a_n = Slerp((1.0/3.0), b_rotate_n, b_slerp_a);

					//b_slerp_b = b_rotate_n1 - b_rotate_n + b_rotate_n1;
					b_slerp_b = Double(b_rotate_n, b_rotate_n1);
					//b_b_n_ = 1/2 * (b_rotate_n2 + b_slerp_b);
					b_b_n_ = Slerp(0.5, b_slerp_b, b_rotate_n2);
					//b_b_n1 = (b_b_n_ - b_rotate_n1) * 1/3 + b_rotate_n1;
					//b_b_n1 = b_rotate_n1 - b_b_n1 + b_rotate_n1;
					b_b_n1 = Slerp(-(1.0/3.0), b_rotate_n1, b_b_n_);
				}
				else
				{
					// next_keyframe > inputLength
					// doesn't have next frame
					//b_rotate_n_1 = pre_posture->bone_rotation[bone];
					Euler2Quaternion(pre_posture->bone_rotation[bone].p, b_rotate_n_1);
					//b_slerp_a = b_rotate_n + b_rotate_n - b_rotate_n_1;
					b_slerp_a = Double(b_rotate_n_1, b_rotate_n);
					//b_a_n_ = 1/2 * (b_slerp_a + b_rotate_n1);
					b_a_n_ = Slerp(0.5, b_slerp_a, b_rotate_n1);
					//b_a_n = (b_a_n_ - b_rotate_n) * 1/3 + b_rotate_n;
					b_a_n = Slerp((1.0/3.0), b_rotate_n, b_a_n_);

					//b_slerp_b = b_rotate_n + b_rotate_n - b_rotate_n_1;
					b_slerp_b = Double(b_rotate_n_1, b_rotate_n);
					//b_b_n1 = (b_slerp_b - b_rotate_n1) * 1/3 + b_rotate_n1;
					b_b_n1 = Slerp((1.0/3.0), b_rotate_n1, b_slerp_b);
				}
				Quaternion<double> res = DeCasteljauQuaternion(t, b_rotate_n, b_a_n, b_b_n1, b_rotate_n1);
				Quaternion2Euler(res, interpolatedPosture.bone_rotation[bone].p);
			}

			pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for(int frame=startKeyframe+1; frame<inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
	double r[9];
	Euler2Rotation(angles, r);
	q = q.Matrix2Quaternion(r);
	//q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
	//q.Normalize();
// 	if (q.Gets() == q.Gets() && q.Getx() == q.Getx() && q.Gety() == q.Gety() && q.Getz() == q.Getz())
// 	{
// 	} else
// 		printf("q2e  error***********************************************\n");
	
	double r[9];
// 	for (int i = 0; i < 9; i ++)
// 	{
// 		if (r[i] == r[i]){}
// 		else
// 			printf("r9  error***********************************************\n");
// 	}
	
	q.Quaternion2Matrix(r);
	Rotation2Euler(r, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
	// cos(theta) = q1 * q2 = s1s2 + x1x2 + y1y2 + z1z2
	double cos_theta = qStart.Gets()*qEnd_.Gets() + qStart.Getx()*qEnd_.Getx()
					 + qStart.Gety()*qEnd_.Gety() + qStart.Getz()*qEnd_.Getz();

	// the accuracy of windows system is too bad!!!!! cos_theta could be 1.0001! F***!
	if (cos_theta > 1)
	{
		return qStart;
	}

	double theta = acos(cos_theta);
// 	if (theta == theta)
// 	{
// 	}
// 	else
// 		printf(" theta error *************");
	if (theta == 0) return qStart;

	// SLERP(q1,q2,u) = sin((1-u)t) / sin(t) * q1 + sin(ut) / sin(t) * q2;
	double sin_theta = sin(theta);
	Quaternion<double> result;

	result = sin((1-t)*theta) / sin_theta * qStart + sin(t*theta) / sin_theta * qEnd_;

// 	if (result.Gets() == result.Gets() && result.Getx() == result.Getx() && result.Gety() == result.Gety() && result.Getz() == result.Getz())
// 	{
// 	}else
// 		printf("slerp error******************************");
	

	//result.Normalize();

	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  result = Slerp(2, p, q);

  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
	/* Course Slide:
	Q0 = Slerp(P0,P1,t) Q1 = Slerp(P1,P2,t) Q2 = Slerp(P2,P3,t) 
	R0 = Slerp(Q0,Q1,t) R1 = Slerp(Q1,Q2,t) 
	P(t)= Slerp(R0,R1,t)
	*/
    vector result;
	vector q0, q1, q2;
	vector r0, r1;
	q0 = p0*(1-t) + p1*t;
	q1 = p1*(1-t) + p2*t;
	q2 = p2*(1-t) + p3*t;

	r0 = q0*(1-t) + q1*t;
	r1 = q1*(1-t) + q2*t;

	result = r0*(1-t) + r1*t;

    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
	/* Course Slide:
	Q0 = Slerp(P0,P1,t) Q1 = Slerp(P1,P2,t) Q2 = Slerp(P2,P3,t) 
	R0 = Slerp(Q0,Q1,t) R1 = Slerp(Q1,Q2,t) 
	P(t)= Slerp(R0,R1,t)
	*/
    Quaternion<double> result;
	Quaternion<double> q0, q1, q2;
	Quaternion<double> r0, r1;

	q0 = Slerp(t, p0, p1);
	q1 = Slerp(t, p1, p2);
	q2 = Slerp(t, p2, p3);

	r0 = Slerp(t, q0, q1);
	r1 = Slerp(t, q1, q2);

	result = Slerp(t, r0, r1);

    return result;
}

