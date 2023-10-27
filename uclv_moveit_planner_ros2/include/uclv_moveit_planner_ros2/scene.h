#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace uclv
{
    class SceneMoveIt
    {
    public:
        // Constructor
        SceneMoveIt(rclcpp::Node::SharedPtr node, bool clear_scene = true)
        {
            // Initialize the planning scene interface
            planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
            node_ = node; //std::make_shared<rclcpp::Node>(node);
            if (clear_scene)
                clearScene();
        }

        // Distructor
        ~SceneMoveIt()
        {
            clearScene();
        }

        void clearScene()
        {
            auto objects_map = planning_scene_interface_->getObjects();
            std::vector<std::string> obj_keys;
            for (auto const &element : objects_map)
            {
                obj_keys.push_back(element.first);
            }
            planning_scene_interface_->removeCollisionObjects(obj_keys);
            node_->get_clock()->sleep_for(std::chrono::seconds(1));
        }

        void addCollisionObjectBOX(double dim_x, double dim_y, double dim_z,
                                   geometry_msgs::msg::PoseStamped pose, const std::string &obj_id)
        {

            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = pose.header.frame_id;
            collision_object.id = obj_id;

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = dim_x;
            primitive.dimensions[primitive.BOX_Y] = dim_y;
            primitive.dimensions[primitive.BOX_Z] = dim_z;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose.pose);
            collision_object.operation = collision_object.ADD;

            planning_scene_interface_->applyCollisionObject(collision_object);
            node_->get_clock()->sleep_for(std::chrono::seconds(1));
        }

        auto createCollisionObjFromMesh(const std::string &obj_id, const geometry_msgs::msg::PoseStamped &pose_obj, const std::string &mesh_path, float mesh_scale)
        {
            moveit_msgs::msg::CollisionObject object_to_attach;
            Eigen::Vector3d scale(mesh_scale, mesh_scale, mesh_scale);

            object_to_attach.id = obj_id;

            // La posizione del box è definita rispetto alla terna world
            object_to_attach.header.frame_id = pose_obj.header.frame_id;

            shapes::Mesh *m = shapes::createMeshFromResource("file://" + mesh_path, scale);

            shape_msgs::msg::Mesh mesh;
            shapes::ShapeMsg mesh_msg;
            shapes::constructMsgFromShape(m, mesh_msg);
            mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

            object_to_attach.meshes.resize(1);
            object_to_attach.mesh_poses.resize(1);
            object_to_attach.meshes[0] = mesh;
            object_to_attach.mesh_poses[0] = pose_obj.pose;

            object_to_attach.meshes.push_back(mesh);
            object_to_attach.mesh_poses.push_back(object_to_attach.mesh_poses[0]);

            object_to_attach.operation = object_to_attach.ADD;

            planning_scene_interface_->applyCollisionObject(object_to_attach);
            return object_to_attach;
        }



    private:
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
        rclcpp::Node::SharedPtr node_;
    };
} // namespace uclv