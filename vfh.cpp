#include<iostream>
#include<vector>
#include<fstream>

#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>



using namespace std;
using namespace pcl;


class CPoint3f{
	private:
		float x;
		float y;
		float z;

		float nx;
		float ny;
		float nz;

	public:
		CPoint3f(float _x = 0,float _y = 0, float _z = 0, float _nx = 0, float _ny = 0, float _nz = 0 ) : x(_x), y(_y), z(_z), nx(_nx), ny(_ny), nz(_nz){}

		friend void average_of_points(const vector<CPoint3f> &set_of_points, CPoint3f &average);

		friend CPoint3f operator- (const CPoint3f &p1, const CPoint3f &p2);

		void print_point3f();
};


CPoint3f CPoint3f::operator- (const CPoint3f &p1, const CPoint3f &p2)
{
	return CPoint3f(p1.x-p2.x,p1.y-p2.y);
}

void CPoint3f::print_point3f()
{
	cout << "{" << x << "," << y << "," << z <<  "}" << "{" << nx << "," << ny << "," <<nz << "}" << endl; 
}


int user_data;

void
downsample (PointCloud<PointXYZRGB>::Ptr &points, float leaf_size, PointCloud<PointXYZRGB>::Ptr &downsampled_out )
{
	VoxelGrid<PointXYZRGB> vox_grid;
	vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	vox_grid.setInputCloud(points);
	vox_grid.filter(*downsampled_out);	
}


void
compute_surface_normals(PointCloud<PointXYZRGB>::Ptr &points, float  normal_radius, PointCloud<Normal>::Ptr &normals_out)
{
	NormalEstimation<PointXYZRGB,Normal> norm_est;
	norm_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr ( new search::KdTree<PointXYZRGB> ));
	norm_est.setRadiusSearch(normal_radius);
	norm_est.setInputCloud(points);
	norm_est.compute(*normals_out);	
}

void
visualize_normals(const PointCloud<PointXYZRGB>::Ptr points,
				  const PointCloud<PointXYZRGB>::Ptr normal_points,
				  const PointCloud<Normal>::Ptr normals)
{
	visualization::PCLVisualizer viz;
	viz.setBackgroundColor (0.26, 0.95, 0.75);
	viz.addPointCloud(points, "points");
	//viz.addPointCloud(normals, "normals");
	viz.addPointCloud(normal_points, "normal_points");
	viz.addPointCloudNormals<PointXYZRGB,Normal> (normal_points, normals, 1, 0.01f,"normals",0);  // OBS  EYE!! 
	viz.spin();

}


    
void 
viewerOneOff (visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.26, 0.95, 0.75);
    PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    cout << "i only run once" << endl;    
}
    
void 
viewerPsycho (visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}



void
write_points_normals_data_on_txt_file(const PointCloud<PointXYZRGB>::Ptr &cloud, const PointCloud<Normal>::Ptr &normals)
{
	ofstream fout("../data_points_and_normals.txt");

	PointCloud<PointXYZRGB>::iterator it_c = cloud->begin();
    for( PointCloud<Normal>::iterator it_n = normals->begin(); it_n != normals->end(); it_n++ )
    {
    	fout << it_c->x << " " <<  it_c->y << " " <<  it_c->z << " " << it_n->normal_x  << " " << it_n->normal_y << " " << it_n->normal_z << endl;    	
    	it_c++; 
    }
    fout.close();
}


void
read_points_normals_data_trhough_txt_file(const char* path, vector<CPoint3f> &points)
{
	ifstream fin(path);
	if(!fin.is_open())
	{
		cout << "Error opening the file .txt" << endl;
		return;
	}
	float x,y,z,nx,ny,nz;
	while(fin>>x>>y>>z>>nx>>ny>>nz)
	{
		points.push_back( CPoint3f(x,y,z,nx,ny,nz) );
		//cout << x << " " << y << " " << z  << " " << xn << " " << yn  << " " << zn  << " " << endl;		
	}
	fin.close();

}


void
average_of_points(const vector<CPoint3f> &set_of_points_input, CPoint3f &average_output)
{
	float _cx = 0;
	float _cy = 0;
	float _cz = 0;

	for(int i = 0; i < set_of_points_input.size(); i++)
	{
		_cx = _cx + set_of_points[i].x;
		_cy = _cy + set_of_points[i].y;
		_cz = _cz + set_of_points[i].z;
	}
	average_output = CPoint3f(_cx,_cy,_cz);
}




bool
vfh_signature(const vector<CPoint3f> &point_cloud_input, const CPoint3f &view_point_input, vector<CPoint3f> &descriptor_output)
{
	vector<CPoint3f> _view_point_directions;
	vector<float> _alpha;     // 45 bins of <0-80>
	vector<float> _phi;       // 45 bins of <0-80>
	vector<float> _theta;     // 45 bins of <0-80>
	vector<float> _distances; // 45 bins of <0-80>
	vector<float> _vp_angles; // 128
	CPoint3f _centroid;

	average_of_points(point_cloud,_centroid);

	for(int i = 0; i < point_cloud.size(); i++)
	{
		_view_point_directions[i] = point_cloud_input[i] - view_point_input; // overload the subtractio operations
	}

	
	




	return true; 
}



int
main()
{
	vector<float> descriptor;
	//PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::io::loadPCDFile ("../bunny.pcd", *cloud);       
    //PointCloud< PointXYZRGB >::Ptr ds(new PointCloud<PointXYZRGB>);
    //PointCloud< Normal >::Ptr normals(new PointCloud<Normal>);
    //const float voxel_grid_leaf_size = 0.001; // Define the discretization
    //downsample(cloud, voxel_grid_leaf_size, ds);
    //const float normal_radius = 0.03;
    //compute_surface_normals ( ds, normal_radius, normals);
    //write_points_normals_data_on_txt_file( cloud, normals ); 
    //pcl::io::savePCDFileASCII("../bunnyNormals.pcd",*normals);
    //visualize_normals(cloud, ds, normals);
  	vector<CPoint3f> data;
  	vector<CPoint3f> descriptor_vfh;
  	CPoint3f view_point = CPoint3f(3.0f,1.0f,4.5f);

	read_points_normals_data_trhough_txt_file("../data_points_and_normals.txt", data);
	for( int i = 0; i < data.size(); i++ )
	{
		data[i].print_point3f();
	}

	vfh_signature(data,view_point,descriptor_vfh);


    //visualization::CloudViewer viewer("Cloud Viewer");
    //viewer.showCloud(cloud);
    //viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //viewer.runOnVisualizationThread (viewerPsycho);    

    /*
    while (!viewer.wasStopped ())
    {
    	user_data++;
    }
    */
      
	return 0;
}
