#include <iostream>
#include<string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>

class PointHeightMethod {
public:
    PointHeightMethod(std::string &base_name, std::string &soil_name, double g_size) :
        base (new pcl::PointCloud<pcl::PointXYZ>()),
        soil (new pcl::PointCloud<pcl::PointXYZ>()),
        tree_soil (new pcl::KdTreeFLANN<pcl::PointXYZ>()),
        tree_base (new pcl::KdTreeFLANN<pcl::PointXYZ>())
        {
            load_cloud(base_name, *base);
            load_cloud(soil_name, *soil);
            grid_size=g_size;
            search_size = grid_size/2;
            soil_volume = estimation_volume();
            save_cloud("save_cloud.ply", save);

            //pcl::PointCloud<pcl::PointXYZ> base_max_points;
            //pcl::PointCloud<pcl::PointXYZ> soil_max_points;
            //pcl::PointCloud<pcl::PointXYZ> search_points;
            //tree_base->setInputCloud(base);
            //tree_soil->setInputCloud(soil);
            //pcl::PointXYZ minPt, maxPt;
            //pcl::getMinMax3D(*base,minPt,maxPt);
            //pcl::PointXYZ search_point;
            //int sss=0;int bbb=0;int ccc=0;
            /*
            for(search_point.x = minPt.x; search_point.x<maxPt.x; search_point.x+=grid_size){
                sss++;
                //std::cout<<"0";
                for(search_point.y = minPt.y; search_point.y<maxPt.y; search_point.y+=grid_size){
                    bbb++;
                    pcl::PointXYZ base_max_point;
                    pcl::PointXYZ soil_max_point;
                    bool base_find = false;
                    bool soil_find = false;
                    for(search_point.z = minPt.z; search_point.z < maxPt.z; search_point.z += search_size / 2){
                        
                        ccc++;
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquareDistance;
                        unsigned int max_nn = 1;
                        search_points.push_back(search_point);
                        tree_base->radiusSearch(search_point, search_size, pointIdxRadiusSearch, pointRadiusSquareDistance);
                        if(pointIdxRadiusSearch.size() > 0){
                            base_find = true;
                            base_max_point = base->points[pointIdxRadiusSearch[0]];
                        }
                    }
                    if (base_find == true){
                        base_max_points.push_back(base_max_point);
                        
                        for(search_point.z = minPt.z; search_point.z<maxPt.z+3; search_point.z+=search_size/2){
                            std::vector<int> pointIdxRadiusSearch;
                            std::vector<float> pointRadiusSquareDistance;
                            unsigned int max_nn = 1;
                            tree_soil->radiusSearch(search_point, search_size, pointIdxRadiusSearch, pointRadiusSquareDistance);
                            if(pointIdxRadiusSearch.size() > 0){
                                soil_find = true;
                                soil_max_point = soil->points[pointIdxRadiusSearch[0]];
                            }
                        }
                    
                    }
                    
                    if (soil_find == true){
                        soil_max_points.push_back(soil_max_point);
                        double z_difference = soil_max_point.z - base_max_point.z;
                        soil_volume+=(grid_size*grid_size)*z_difference;
                    }
                }

            }
            
            std::cout<<sss<<std::endl;
            std::cout<<bbb<<std::endl;
            std::cout<<ccc<<std::endl;
            */
            //save_cloud("search.ply",search_points);
            //save_cloud("base_max.ply",base_max_points);
            //save_cloud("soil_max.ply",soil_max_points);
            std::cout<<"soil_volume = "<<soil_volume<<" m^3"<<std::endl;            

        }
    pcl::PointCloud<pcl::PointXYZ>::Ptr base;
    pcl::PointCloud<pcl::PointXYZ>::Ptr soil;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_soil;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_base;
    pcl::PointCloud<pcl::PointXYZ> save;

    double grid_size;
    double search_size;
    double soil_volume = 0.0;
    //float resolution = 0.00001;

    bool load_cloud(std::string cloud_name, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void save_cloud(std::string save_cloud_name, pcl::PointCloud<pcl::PointXYZ> cloud);
    std::vector<int> radius_search(pcl::PointXYZ search_point, pcl::KdTreeFLANN<pcl::PointXYZ>& tree);
    pcl::PointXYZ search_maxpoint(pcl::KdTreeFLANN<pcl::PointXYZ>& tree, pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ search_point, double minPt_z, double maxPt_z);
    double estimation_volume();
};

bool PointHeightMethod::load_cloud(std::string cloud_name, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if(pcl::io::load(cloud_name, cloud)){
        return false;
    }
    std::vector<int> nanIndexes;
    pcl::removeNaNFromPointCloud(cloud, cloud, nanIndexes);
    std::string load_cloud_name = cloud_name;
    pcl::io::load(load_cloud_name, cloud);

    return true;
}

void PointHeightMethod::save_cloud(std::string save_cloud_name, pcl::PointCloud<pcl::PointXYZ> cloud)
{
    std::vector<int> nanIndexes;
    pcl::removeNaNFromPointCloud(cloud, cloud, nanIndexes);
    pcl::io::savePLYFile(save_cloud_name, cloud);
}

std::vector<int> PointHeightMethod::radius_search(pcl::PointXYZ search_point, pcl::KdTreeFLANN<pcl::PointXYZ>& tree){

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquareDistance;
    unsigned int max_nn = 1;

    tree.radiusSearch(search_point, search_size, pointIdxRadiusSearch, pointRadiusSquareDistance);

    return pointIdxRadiusSearch;
}

pcl::PointXYZ  PointHeightMethod::search_maxpoint(pcl::KdTreeFLANN<pcl::PointXYZ>& tree, pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointXYZ search_point, double minPt_z, double maxPt_z){

    pcl::PointXYZ max_point;
    max_point.x=0;max_point.y=0;max_point.z=0;

    for(search_point.z = minPt_z; search_point.z < maxPt_z; search_point.z += search_size / 2){

        std::vector<int> pointIdxRadiusSearch = radius_search(search_point, tree);
        
        if (pointIdxRadiusSearch.size() > 0)
            max_point = cloud.points[pointIdxRadiusSearch[0]];
        
    }

    return max_point;
}

double PointHeightMethod::estimation_volume()
{
    double soil_volume = 0;

    pcl::PointCloud<pcl::PointXYZ> base_max_points;
    pcl::PointCloud<pcl::PointXYZ> soil_max_points;
    pcl::PointCloud<pcl::PointXYZ> search_points;
    
    pcl::PointXYZ minPt, maxPt;
    pcl::PointXYZ search_point;
    pcl::PointXYZ base_max_point;
    pcl::PointXYZ soil_max_point;

    tree_base->setInputCloud(base);
    tree_soil->setInputCloud(soil);

    pcl::getMinMax3D(*base,minPt,maxPt);
    for(search_point.x = minPt.x; search_point.x<maxPt.x; search_point.x+=grid_size){
        for(search_point.y = minPt.y; search_point.y<maxPt.y; search_point.y+=grid_size){

            double min_z=minPt.z,max_z=maxPt.z;
            base_max_point = search_maxpoint(*tree_base, *base, search_point, min_z, max_z);
            if(!(base_max_point.x==0&&base_max_point.y==0&&base_max_point.z==0)){
                save.push_back(base_max_point);
                double base_max_z = base_max_point.z;
                soil_max_point = search_maxpoint(*tree_soil, *soil, search_point, base_max_z, max_z+3);
                double z_difference = soil_max_point.z - base_max_point.z;
                soil_volume+=(grid_size*grid_size)*z_difference;

            }
        }
    }
    return soil_volume;
}


int main(int argc, char* argv[]){
    
    if (argc != 4 )
    {
        std::cerr << "ERROR: Syntax is ./point_height_method <base ply file> <target ply file> <grid size>"<<std::endl;
        std::cerr << "EXAMPLE: ./point_height_method base.ply soil.ply 0.01" <<std::endl;
        return -1;
    }

    std::string base_path(argv[1]);
    std::string soil_path(argv[2]);
    double grid_size = atof(argv[3]);
    PointHeightMethod point_height_method(base_path, soil_path, grid_size);

    return 0;
}
