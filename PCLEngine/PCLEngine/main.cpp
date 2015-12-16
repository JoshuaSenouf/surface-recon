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

PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);


int _tmain(int argc, TCHAR** argv)
{
	static int verbose_flag;
	int c;

	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
	io::loadPCDFile (argv[1], *cloud);

	while (1)
	{		
		static struct option long_options[] =
		{
			{_T("verbose"), ARG_NONE, &verbose_flag, 1},
			{_T("brief"),   ARG_NONE, &verbose_flag, 0},
			{_T("add"),     ARG_NONE, 0, _T('a')},
			{_T("append"),  ARG_NONE, 0, _T('b')},
			{_T("delete"),  ARG_REQ,  0, _T('d')},
			{_T("create"),  ARG_REQ,  0, _T('c')},
			{_T("file"),    ARG_REQ, 0 , _T('f')},
			{ ARG_NULL , ARG_NULL , ARG_NULL , ARG_NULL }
		};

		int option_index = 0;
		c = getopt_long(argc, argv, _T("fnsgpm"), long_options, &option_index);

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

		case _T('f'):
			_tprintf(_T("FlipViewpoint activated...\n"));
			flipViewpoint = true;
			break;

		case _T('n'): 
			{
				_tprintf(_T("Normals computing...\n"));

				pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normE;
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

				tree->setInputCloud (cloud);
				normE.setInputCloud (cloud);
				normE.setSearchMethod (tree);
				normE.setKSearch (5);

				pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
				normE.compute (*normals);

				Eigen::Vector4f centroid;
				compute3DCentroid (*cloud, centroid);
				normE.setViewPoint (centroid[0], centroid[1], centroid[2]);

				if(flipViewpoint) {
					for (size_t i = 0; i < normals->size (); ++i) {
						normals->points[i].normal_x *= -1;
						normals->points[i].normal_y *= -1;
						normals->points[i].normal_z *= -1;
					}
				}

				pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

				io::savePLYFile ("cloud_with_normals.ply", *cloud_with_normals);
				break;
			}

		case _T('s'): 
			{
				_tprintf(_T("Moving Least Squares reconstruction...\n"));
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
				pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

				mls.setComputeNormals (true);
				mls.setInputCloud (cloud);
				mls.setPolynomialFit (true);
				mls.setSearchMethod (tree);
				mls.setSearchRadius (5);
				mls.process (*cloud_with_normals);

				if(flipViewpoint) {
					for (size_t i = 0; i < cloud_with_normals->size (); ++i) {
						cloud_with_normals->points[i].normal_x *= -1;
						cloud_with_normals->points[i].normal_y *= -1;
						cloud_with_normals->points[i].normal_z *= -1;
					}
				}

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
				gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
				gp3.setMinimumAngle(M_PI/18); // 10 degrees
				gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
				gp3.setNormalConsistency(false);

				gp3.setInputCloud (cloud_with_normals);
				gp3.setSearchMethod (tree2);
				gp3.reconstruct (triangles);

				std::vector<int> parts = gp3.getPartIDs();
				std::vector<int> states = gp3.getPointStates();

				io::savePLYFile ("output.ply", triangles);
				break;
			}

		case _T('p'): 
			{
				_tprintf(_T("Poisson reconstruction...\n"));

				Poisson<PointNormal> poissonReconstruction;
				poissonReconstruction.setDepth (8);
				poissonReconstruction.setInputCloud (cloud_with_normals);

				PolygonMesh outputMesh;
				poissonReconstruction.reconstruct (outputMesh);

				io::savePLYFile ("output.ply", outputMesh);
				break;
			}

		case _T('m'): 
			{
				_tprintf(_T("Marching Cubes reconstruction...\n"));
				MarchingCubesHoppe<PointNormal> hoppe;
				PolygonMesh::Ptr triangles(new PolygonMesh);

				hoppe.setIsoLevel (0.5);
				hoppe.setGridResolution (50, 30, 50);
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