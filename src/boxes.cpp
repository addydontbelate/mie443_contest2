#include <ros/package.h>
#include "boxes.h"

bool Boxes::load_coords() 
{
    std::string filePath = ros::package::getPath("mie443_contest2") + 
                           std::string("/boxes_database/gazebo_coords.xml");
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    
    if (fs.isOpened()) 
    {
        cv::FileNode node;
        cv::FileNodeIterator it, end;
        std::vector<float> coordVec;
        std::string coords_xml[5] = {"coordinate1", "coordinate2", "coordinate3", "coordinate4",
                                     "coordinate5"};
        for(const auto & i : coords_xml) 
        {
            node = fs[i];
            if(node.type() != cv::FileNode::SEQ) 
            {
                std::cout << "XML ERROR: Data in " << i 
                          << " is improperly formatted - check input.xml" << std::endl; 
            } 
            else 
            {
                it = node.begin();
                end = node.end();
                coordVec = std::vector<float>();
                
                for(int j = 0; it != end; ++it, ++j)
                    coordVec.push_back((float)*it);
                
                if (coordVec.size() == 3) 
                {
                    coords.push_back(coordVec);
                } 
                else 
                {
                    std::cout << "XML ERROR: Data in " << i 
                              << " is improperly formatted - check input.xml" << std::endl; 
                }
            }
        }
        
        if (coords.empty()) 
        {
            std::cout << "XML ERROR: Coordinate data is improperly formatted - check input.xml" 
                  << std::endl;
            return false;
        }
    } 
    else 
    {
        std::cout << "Could not open XML - check FilePath in " << filePath << std::endl;
        return false;
    }
    
    return true;
}

bool Boxes::load_templates() 
{
    std::string filePath = ros::package::getPath("mie443_contest2") + 
                           std::string("/boxes_database/templates.xml");
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    
    if (fs.isOpened()) 
    {
        cv::FileNode node = fs["templates"];
        
        if (!(node.type() == cv::FileNode::SEQ || node.type() == cv::FileNode::STRING)) 
        { 
            std::cout << "XML ERROR: Image data is improperly formatted in " << filePath 
                      << std::endl;
            return false;
        }
        
        std::string img_path;
        for(const auto & it: node)
        {
            img_path = ros::package::getPath("mie443_contest2") + 
                        std::string("/boxes_database/") + 
                        std::string(it);
            templates.push_back(cv::imread(img_path, cv::IMREAD_GRAYSCALE));
        }        
    } 
    else 
    {
        std::cout << "XML ERROR: Could not open " << filePath << std::endl;
        return false;
    }
    
    return true;
}
