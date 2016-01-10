/* PCLEngine for PCL

Date: 09/01/2016 - Joshua Senouf - Initial Release
Version: 1.0
Comment: Support of Normals Estimation/MLS/Greedy/Poisson/Marching Cubes
License: LGPL

Revisions:

09/01/2016 - Joshua Senouf - Initial Release


**DISCLAIMER**
THIS MATERIAL IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING, BUT Not LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE, OR NON-INFRINGEMENT. SOME JURISDICTIONS DO NOT ALLOW THE
EXCLUSION OF IMPLIED WARRANTIES, SO THE ABOVE EXCLUSION MAY NOT
APPLY TO YOU. IN NO EVENT WILL I BE LIABLE TO ANY PARTY FOR ANY
DIRECT, INDIRECT, SPECIAL OR OTHER CONSEQUENTIAL DAMAGES FOR ANY
USE OF THIS MATERIAL INCLUDING, WITHOUT LIMITATION, ANY LOST
PROFITS, BUSINESS INTERRUPTION, LOSS OF PROGRAMS OR OTHER DATA ON
YOUR INFORMATION HANDLING SYSTEM OR OTHERWISE, EVEN If WE ARE
EXPRESSLY ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*/

#include "getopt.h"
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <string.h>
#include <stdio.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>	
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <iostream>
#include <vector>

using namespace pcl;
using namespace std;


bool flipViewpoint = false;
bool normalOMP = false;
bool upsamplingMLS = false;
string tempNO;

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);


int _tmain(int argc, TCHAR** argv)
{
	static int verbose_flag;
	int c;

	while (1)
	{		
		static struct option long_options[] =
		{
			{_T("verbose"), ARG_NONE, &verbose_flag, 1},
			{_T("brief"),   ARG_NONE, &verbose_flag, 0},
			{_T("fvp"),     ARG_NONE, 0, _T('a')},			// Flip the viewPoint for normals computation
			{_T("omp"),     ARG_NONE, 0, _T('b')},			// Use the OMP version for normals computation
			{_T("up"),     ARG_NONE, 0, _T('c')},			// Upsample the cloud point
			{_T("no"),  ARG_REQ, 0, _T('d')},				// Specify if the PCD file in argument has normals (= true) or not (= false) and load the file with the proper datatype -- MANDATORY
			{_T("nc"),  ARG_NONE, 0, _T('e')},				// Compute normals
			{_T("mls"),  ARG_NONE,  0, _T('f')},			// Moving Least Squares for normals/upsampling
			{_T("gr"),  ARG_NONE,  0, _T('g')},				// Greedy reconstruction
			{_T("po"),    ARG_NONE, 0 , _T('h')},			// Poisson reconstruction
			{_T("mar"),    ARG_NONE, 0 , _T('i')},			// Marching Cubes reconstruction
			{ ARG_NULL , ARG_NULL , ARG_NULL , ARG_NULL }
		};

		int option_index = 0;
		c = getopt_long(argc, argv, _T("abcd:efghi"), long_options, &option_index);

		if (c == -1)
			break;

		switch (c)
		{
		case 0:
			if (long_options[option_index].flag != 0)
				break;
			_tprintf (_T("option %s"), long_options[option_index].name);
			if (optarg)
				_tprintf (_T(" with arg %s"), optarg);
			_tprintf (_T("\n"));
			break;

		case _T('a'):
			{
				_tprintf(_T("FlipViewpoint activated...\n"));
				flipViewpoint = true;
				break;
			}

		case _T('b'):
			{
				_tprintf(_T("NormalOMP activated...\n"));
				normalOMP = true;
				break;
			}

		case _T('c'):
			{
				_tprintf(_T("Upsampling activated...\n"));
				upsamplingMLS = true;
				break;
			}

		case _T('d'):
			{
				tempNO = optarg;
				if (tempNO == "true")	// If the PCD file contains normals, we use the PointCloud<PointNormal> datatype, if not, we use the PointCloud<PointXYZ>
					io::loadPCDFile (argv[1], *cloud_with_normals);
				else io::loadPCDFile (argv[1], *cloud);
				break;
			}

		case _T('e'): 
			{
				_tprintf(_T("Normals computing...\n"));

				pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normE;
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); // We use a K-Dimensional Tree

				if (normalOMP) {
					NormalEstimationOMP<PointXYZ, Normal> normE;
					normE.setNumberOfThreads (4);	// Set the number of threads assigned to the process if the OpenMP method has been chosen
				}

				tree->setInputCloud (cloud);	// Set the input cloud that will be used in the tree search/normals computing
				normE.setInputCloud (cloud);
				normE.setSearchMethod (tree);
				normE.setKSearch (5);	// Set the search depth in the K-Dimensional Tree

				pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
				normE.compute (*normals);

				Eigen::Vector4f centroid;
				compute3DCentroid (*cloud, centroid);
				normE.setViewPoint (centroid[0], centroid[1], centroid[2]);

				if(flipViewpoint) {
					for (size_t i = 0; i < normals->size (); ++i) {	// We flip the normals the normals of each point after computation of their centroid
						normals->points[i].normal_x *= -1;
						normals->points[i].normal_y *= -1;
						normals->points[i].normal_z *= -1;
					}
				}

				pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);		//We fuse the two PointCloud in one that can be used by the reconstruction methods

				io::savePLYFile ("cloud_with_normals.ply", *cloud_with_normals);	// Export in the resulting cloud in a .PLY file, viewable in Meshlab
				io::savePCDFile ("cloud_with_normals.pcd", *cloud_with_normals);	// Export in the resulting cloud in a .PCD file
				break;
			}

		case _T('f'): // Can be buggy with Greedy Reconstruction, will depend on the input cloud point, then adjust the search radius if crash occured
			{
				_tprintf(_T("Moving Least Squares reconstruction...\n"));
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
				pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

				mls.setComputeNormals (true);
				mls.setInputCloud (cloud);
				mls.setPolynomialFit (true);	// Set the computation of normals to be more accurate (and longer)
				mls.setPolynomialOrder (2);
				mls.setSearchMethod (tree);
				mls.setSearchRadius (5);

				mls.process (*cloud_with_normals);

				//	Upsampling buggy at the moment, the cloud_smoothed need to have as much elements as cloud_with_normals in the end if we choose this option
				//
				//if (upsamplingMLS) {
				//	PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());

				//	mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
				//	mls.setUpsamplingRadius (5);
				//	mls.setUpsamplingStepSize (3);

				//	mls.process (*cloud_smoothed);
				//}


				if(flipViewpoint) {
					for (size_t i = 0; i < cloud_with_normals->size (); ++i) {
						cloud_with_normals->points[i].normal_x *= -1;
						cloud_with_normals->points[i].normal_y *= -1;
						cloud_with_normals->points[i].normal_z *= -1;
					}
				}


				io::savePLYFile ("cloud_with_normals.ply", *cloud_with_normals);
				io::savePCDFile ("cloud_with_normals.pcd", *cloud_with_normals);
				break;
			}

		case _T('g'): 
			{
				_tprintf(_T("Greedy reconstruction...\n"));

				pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
				tree2->setInputCloud (cloud_with_normals);

				pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
				pcl::PolygonMesh triangles;

				gp3.setSearchRadius (300);
				gp3.setMu (2.5);
				gp3.setMaximumNearestNeighbors (100);
				gp3.setMaximumSurfaceAngle(M_PI/4);
				gp3.setMinimumAngle(M_PI/18);
				gp3.setMaximumAngle(2*M_PI/3);
				gp3.setNormalConsistency(true);

				gp3.setInputCloud (cloud_with_normals);
				gp3.setSearchMethod (tree2);
				gp3.reconstruct (triangles);

				io::savePLYFile ("output.ply", triangles);
				break;
			}

		case _T('h'): 
			{
				_tprintf(_T("Poisson reconstruction...\n"));

				Poisson<PointNormal> poissonReconstruction;
				poissonReconstruction.setDepth (8);	// Set at which depth the octree will be searched
				poissonReconstruction.setInputCloud (cloud_with_normals);

				PolygonMesh outputMesh;
				poissonReconstruction.reconstruct (outputMesh);

				io::savePLYFile ("output.ply", outputMesh);
				break;
			}

		case _T('i'): 
			{
				_tprintf(_T("Marching Cubes reconstruction...\n"));
				MarchingCubesHoppe<PointNormal> hoppe;
				PolygonMesh::Ptr triangles(new PolygonMesh);

				hoppe.setIsoLevel (0.5);
				hoppe.setGridResolution (50, 30, 50);	// Set the resolution of the 3D grid, the larger it is, the longer it will be to reconstruct the surface
				hoppe.setPercentageExtendGrid (0.1f);
				hoppe.setInputCloud (cloud_with_normals);
				hoppe.reconstruct (*triangles);

				io::savePLYFile ("output.ply", *triangles);
				break;
			}

		case '?':
			break;

		default:
			abort();
		}
	}

	if (verbose_flag)
		_tprintf (_T("verbose flag is set\n"));


	if (optind < argc)
	{
		_tprintf (_T("non-option ARGV-elements: "));
		while (optind < argc) _tprintf (_T("%s "), argv[optind++]);
		_tprintf (_T("\n"));
	}
	return 0;
}