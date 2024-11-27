#include <stdio.h>
#include <math.h>

#define ARM1_LENGTH 220.0  // ��۳�
#define ARM2_LENGTH 180.0  // С�۳�
#define GRIPPER_LENGTH 120.0  // ��׊����
#define MAX_ITER 2000       // ����������
#define LEARNING_RATE 0.05  // ѧϰ�ʣ������ݶ���
#define TOLERANCE 0.1      // ��������
#define PI 3.14159265359

#define DEG_TO_RAD(angleInDegrees) ((angleInDegrees) * PI / 180.0)
#define RAD_TO_DEG(angleInRadians) ((angleInRadians) * 180.0 / PI)

// Ŀ��λ��
typedef struct {
    double x;
    double y;
} Position;

// �؜ڜǶ�
typedef struct {
    double theta1;
    double theta2;
    double theta3;
} Angles;

// �����е��ĩ�˵ĵ�ǰλ��
Position forward_kinematics(Angles angles) {
    Position pos;
    double theta1_rad = DEG_TO_RAD(angles.theta1);
    double theta2_rad = DEG_TO_RAD(angles.theta2);
    double theta3_rad = DEG_TO_RAD(angles.theta3);

    // ����λ�� A, B, C
    double Ax = ARM1_LENGTH * cos(theta1_rad);
    double Ay = ARM1_LENGTH * sin(theta1_rad);

    double Bx = Ax + ARM2_LENGTH * cos(theta1_rad + theta2_rad);
    double By = Ay + ARM2_LENGTH * sin(theta1_rad + theta2_rad);

    double Cx = Bx + GRIPPER_LENGTH * cos(theta1_rad + theta2_rad + theta3_rad);
    double Cy = By + GRIPPER_LENGTH * sin(theta1_rad + theta2_rad + theta3_rad);

    pos.x = Cx;
    pos.y = Cy;
    return pos;
}

// ����Ŀ��λ�ú͵�ǰĩ��λ��֮������
double compute_error(Position target, Position current) {
    return sqrt((target.x - current.x) * (target.x - current.x) + (target.y - current.y) * (target.y - current.y));
}

// ʹ���ݶ������������؜ڜǶ�
void inverse_kinematics(Position target, Angles *angles) {
    for (int i = 0; i < MAX_ITER; i++) {
        // ���㵱ǰ��λ��
        Position current = forward_kinematics(*angles);

        // �������
        double error = compute_error(target, current);
        if (error < TOLERANCE) {
            printf("ѭ��������%d\n", i);
            printf("������Ŀ��λ�ã����Ϊ: %.3f\n", error);
            return;
        }

        // �����ݶȣ���ÿ���Ƕ���ƫ������ʹ����ֵ΢�֣�
        double delta = 1e-6;
        Angles gradients;
        
        // ���� theta1 ���ݶ�
        angles->theta1 += delta;
        Position pos1 = forward_kinematics(*angles);
        gradients.theta1 = (compute_error(target, pos1) - error) / delta;
        angles->theta1 -= delta;

        // ���� theta2 ���ݶ�
        angles->theta2 += delta;
        Position pos2 = forward_kinematics(*angles);
        gradients.theta2 = (compute_error(target, pos2) - error) / delta;
        angles->theta2 -= delta;

        // ���� theta3 ���ݶ�
        angles->theta3 += delta;
        Position pos3 = forward_kinematics(*angles);
        gradients.theta3 = (compute_error(target, pos3) - error) / delta;
        angles->theta3 -= delta;

        // ʹ���ݶ�������ÿ���Ƕ�
        angles->theta1 -= LEARNING_RATE * gradients.theta1;
        angles->theta2 -= LEARNING_RATE * gradients.theta2;
        angles->theta3 -= LEARNING_RATE * gradients.theta3;
    }
    printf("�ﵜ���������������Ϊ: %.3f\n", compute_error(target, forward_kinematics(*angles)));
}

int main() {
    Position target = {-300.0, -150.0};  // Ŀ��λ�� (x, y)
    Angles initial_angles = {15.0, -37.0, 78.0};  // ��ʌ�؜ڜǶ� (theta1, theta2, theta3)
    // Position target = {300.0, 200.0};  // Ŀ��λ�� (x, y)
    // Angles initial_angles = {-0.1, 0.0, 1.0};  // ��ʌ�؜ڜǶ� (theta1, theta2, theta3)

    initial_angles.theta3 -= initial_angles.theta2;
    initial_angles.theta2 -= initial_angles.theta1;

    printf("��ʌ�؜ڜǶ�: theta1=%.2f, theta2=%.2f, theta3=%.2f\n", initial_angles.theta1, initial_angles.theta2, initial_angles.theta3);

    inverse_kinematics(target, &initial_angles);

    initial_angles.theta2 += initial_angles.theta1;
    initial_angles.theta3 += initial_angles.theta2;

    printf("���չ؜ڜǶ�: theta1=%.2f, theta2=%.2f, theta3=%.2f\n", initial_angles.theta1, initial_angles.theta2, initial_angles.theta3);
    return 0;
}