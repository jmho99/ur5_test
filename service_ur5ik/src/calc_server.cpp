#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sstream>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "interfaces_ur5ik/srv/six_theta.hpp"

double m_theta[6] = { 0 };
double pi = 3.141592;
//DHparameter (m)
double m_d1 = 0.104159;
double m_a2 = -0.42500;
double m_a3 = -0.39225;
double m_d4 = 0.10915;
double m_d5 = 0.09465;
double m_d6 = 0.2235;

double T[4][4] = { 0 };

struct customNode {
    double data[6]; // 크기 6인  array
    customNode* next;

    customNode(double val[]) {
        for (int i = 0; i < 6; i++) {
            data[i] = val[i];
        }
        next = nullptr;
    }
};

class LinkedList {
public:
    customNode* head;

    LinkedList() : head(nullptr) {}

    void append(double val[]) {
        customNode* newNode = new customNode(val);
        if (head == nullptr) {
            head = newNode;
        } else {
            customNode* temp = head;
            while (temp->next != nullptr) {
                temp = temp->next;
            }
            temp->next = newNode;
        }
    }

    customNode* getHead() {
        return head;
    }
};


class ArrayServiceServer : public rclcpp::Node {
public:
    ArrayServiceServer() : Node("array_service_server"), current_node(nullptr) {
    service_client_ = this->create_service<interfaces_ur5ik::srv::SixTheta>("service_ik",std::bind(&ArrayServiceServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service client ur5 inverse kinematics");
        // 링크드 리스트 초기화
        double arr1[6] = {0.0, 0.3, 0.3, 3.14, 0.0, 0.0};
        double arr2[6] = {0.05, 0.25, 0.3, 3.14, 0.0, 0.0};
        double arr3[6] = {0.1, 0.3, 0.3, 3.14, 0.0, 0.0};
	double arr4[6] = {0.1, 0.3, 0.3, 3.14, 0.0, 0.0};
	double arr5[6] = {0.1, 0.4, 0.25, 3.14, 0.0, 0.0};
	double arr6[6] = {0.1, 0.45, 0.2, 3.14, 0.0, 0.0};
	double arr7[6] = {0.1, 0.5, 0.25, 3.14, 0.0, 0.0};
	double arr8[6] = {0.1, 0.55, 0.3, 3.14, 0.0, 0.0};
	
        list_.append(arr1);
        list_.append(arr2);
        list_.append(arr3);
        list_.append(arr4);
        list_.append(arr5);
        list_.append(arr6);
        list_.append(arr7);
        list_.append(arr8);
        


        current_node = list_.getHead();
    }


private:
void set_trans(double x, double y, double z, double roll, double pitch, double yaw)
    {
        double XYZ[3][1] = { {x},{y},{z} };

        double Rx[3][3] = { {1, 0, 0},
                            {0, cos(roll), - sin(roll)},
                            {0, sin(roll), cos(roll)} };

        double Ry[3][3] = { {cos(pitch), 0, sin(pitch)},
                            {0, 1, 0},
                            {-sin(pitch), 0, cos(pitch)} };

        double Rz[3][3] = { {cos(yaw), - sin(yaw), 0},
                            {sin(yaw), cos(yaw), 0},
                            {0, 0, 1} };

        double R[3][3] = { 0 };

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                for (int k = 0; k < 3; k++)
                {
                    for (int l = 0; l < 3; l++)
                    {
                        R[row][column] += Rz[row][k] * Ry[k][l] * Rx[l][column];
                    }

                }
            }
        }

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                if (row <= 2 && column <= 2)
                {
                    T[row][column] = R[row][column];
                }
                else if (row <= 2 && column == 3)
                {
                    T[row][column] = XYZ[row][0];
                }
                else if (row == 3 && column <= 2)
                {
                    T[row][column] = 0;
                }
                else
                {
                    T[row][column] = 1;
                }
            }
        }
	}

void cal_theta()
    {
        double p05[4][1] = { 0 };
        double p65[4][1] = { {0},{0},{-m_d6},{1} };

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 1; column++)
            {
                for (int k = 0; k < 4; k++)
                {
                    p05[row][column] += T[row][k] * p65[k][column];
                }
            }

        }

        m_theta[0] = atan2(p05[1][0], p05[0][0]) - acos(m_d4 / sqrt(pow(p05[0][0], 2) + pow(p05[1][0], 2))) + pi / 2;
        m_theta[4] = acos((T[0][3] * sin(m_theta[0]) - T[1][3] * cos(m_theta[0]) - m_d4) / m_d6);
        m_theta[5] = atan2(((-T[0][1] * sin(m_theta[0]) + T[1][1] * cos(m_theta[0])) / sin(m_theta[4])),
            ((T[0][0] * sin(m_theta[0]) - T[1][0] * cos(m_theta[0])) / sin(m_theta[4])));

        double theta234 = atan2(-T[2][2] / sin(m_theta[4]),
            (T[0][2] * cos(m_theta[0]) + T[1][2] * sin(m_theta[0])) / sin(m_theta[4]));
        double T24_b14 = T[0][3] * cos(m_theta[0]) + T[1][3] * sin(m_theta[0])
            - m_d6 * T[0][2] * cos(m_theta[0]) - m_d6 * T[1][2] * sin(m_theta[0])
            + m_d5 * T[1][0] * sin(m_theta[0]) * sin(m_theta[5]) + m_d5 * T[0][1] * cos(m_theta[0]) * cos(m_theta[5])
            + m_d5 * T[0][0] * cos(m_theta[0]) * sin(m_theta[5]) + m_d5 * T[1][1] * sin(m_theta[0]) * cos(m_theta[5]);
        double T24_b24 = T[2][3] - m_d1 - m_d6 * T[2][2] + m_d5 * T[2][1] * cos(m_theta[5]) + m_d5 * T[2][0] * sin(m_theta[5]);

        //theta3 = 2 * atan(sqrt(((a2 + a3) ^ 2 - (b14. ^ 2 + b24. ^ 2)) / ((b14. ^ 2 + b24. ^ 2) - (a2 - a3) ^ 2)))
        m_theta[2] = -acos((pow(T24_b14,2) + pow(T24_b24,2) - pow(m_a2,2) - pow(m_a3,2)) / (2 * m_a2 * m_a3));

        double alpha = atan2(T24_b24, T24_b14);
        double beta = atan2(m_a3 * sin(m_theta[2]), m_a2 + m_a3 * cos(m_theta[2]));
        m_theta[1] = alpha - beta;

        m_theta[3] = theta234 - m_theta[1] - m_theta[2];
    }
    void handle_request(const std::shared_ptr<interfaces_ur5ik::srv::SixTheta::Request> request,     
          std::shared_ptr<interfaces_ur5ik::srv::SixTheta::Response>       response) {
    //auto x = request -> srv_target[0]; auto y = request -> srv_target[1]; auto z = request -> srv_target[2];
    //auto roll = request -> srv_target[3]; auto pitch = request -> srv_target[4]; auto yaw = request -> srv_target[5];

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nx: %f" " \ny : %f" " \nz : %f"
    //            " \nroll : %f" " \npitch : %f" " \nyaw : %f",
    //			x, y, z, roll, pitch, yaw);


    



        if (current_node != nullptr) {
            RCLCPP_INFO(this->get_logger(), "Sending array: [%d, %d, %d, %d, %d, %d]",
                        current_node->data[0], current_node->data[1], current_node->data[2],
                        current_node->data[3], current_node->data[4], current_node->data[5]);
		
		double x = current_node->data[0];
		double y = current_node->data[1];
		double z = current_node->data[2];
		double roll = current_node->data[3];
		double pitch = current_node->data[4];
		double yaw = current_node->data[5];
		
		set_trans(x,y,z,roll,pitch,yaw);
		cal_theta();
                
                response -> srv_theta[0] = m_theta[0];
                response -> srv_theta[1] = m_theta[1];
                response -> srv_theta[2] = m_theta[2];
                response -> srv_theta[3] = m_theta[3];
                response -> srv_theta[4] = m_theta[4];
                response -> srv_theta[5] = m_theta[5];		
                
                current_node = current_node->next;
                
                response ->success = true;
		
        } else {
            RCLCPP_INFO(this->get_logger(), "All commands have been sent.");
                response -> srv_theta[0] = 0;
                response -> srv_theta[1] = 0;
                response -> srv_theta[2] = 0;
                response -> srv_theta[3] = 0;
                response -> srv_theta[4] = 0;
                response -> srv_theta[5] = 0;
                
                response ->success = false;
        }
    
    }
    rclcpp::Service<interfaces_ur5ik::srv::SixTheta>::SharedPtr service_client_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    LinkedList list_;
    customNode* current_node;
};
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("calc_server");

	//rclcpp::Service<interfaces_ur5ik::srv::SixTheta>::SharedPtr server =
		//node -> create_service<interfaces_ur5ik::srv::SixTheta>("service_ik", &handle_request);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set target.");

	rclcpp::spin(std::make_shared<ArrayServiceServer>());
	rclcpp::shutdown();
}

