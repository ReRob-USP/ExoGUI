#pragma once

#include <thread_exo.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#include <declarations_xsens.h>
#pragma GCC diagnostic pop
#include <LowPassFilter2p.h>
#include <xsensLeoUtils.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <utils_plot.hpp>


typedef struct {
	float data[10];
} fromXsensLeo;

class ThreadXsensLeo : public ThreadType {

private:
	fromXsensLeo data;
	std::atomic<bool> plotting;
	PlotWindow pw;

public:

	ThreadXsensLeo() {
		pw = PlotWindow("Units [ u ]", "Time [ s ]", "Xsens Data");
		plotting = false;
	}

	float q0;
	float q1;
	float q2;
	float q3;
	float imus_data[24];
	float states_data[10];
	float rad2deg;
	float euler_k[3];
	float omega_k[3];
	float Ts;
	const float mi0 = 0.36;
	const float Beta = 10.0;
	const Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
	const Eigen::Matrix4f R = Eigen::Matrix4f::Identity() * 2.5e-5;

	Eigen::Vector4f qASGD1_qk;
	Eigen::Vector4f qASGD2_qk;
	Eigen::Vector4f qASGD3_qk;
	Eigen::Matrix4f qASGD1_Pk;
	Eigen::Matrix4f qASGD2_Pk;
	Eigen::Matrix4f qASGD3_Pk;

	Eigen::FullPivLU<Eigen::Matrix4f> Covar1;
	Eigen::FullPivLU<Eigen::Matrix4f> Covar2;
	Eigen::FullPivLU<Eigen::Matrix4f> Covar3;
	Eigen::Matrix4f Q1;
	Eigen::Matrix4f Q2;
	Eigen::Matrix4f Q3;

	Eigen::Quaternionf q12Off;
	Eigen::Quaternionf q23Off;

	Eigen::Vector3f gyro1;
	Eigen::Vector3f gyro2;
	Eigen::Vector3f gyro3;
	Eigen::Vector3f gyro4;
	Eigen::Vector3f acc1;
	Eigen::Vector3f acc2;
	Eigen::Vector3f acc3;
	Eigen::Vector3f acc4;
	Eigen::Vector3f F_obj;
	Eigen::Vector4f GradF;
	Eigen::Vector4f z_k;
	Eigen::Vector3f Zc;
	Eigen::Matrix4f OmG;
	Eigen::Matrix4f Psi;
	Eigen::Matrix4f Kg1;
	Eigen::Matrix4f Kg2;
	Eigen::Matrix4f Kg3;
	Eigen::Matrix4f Qy;
	Eigen::Matrix<float, 3, 4> Jq;
	Eigen::Matrix<float, 4, 3> Xi;
	int offCount;
	float omg_norm;
	float mi;


	void _setup() {

		((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[0].required = true;
		((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[1].required = true;
		((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[2].required = true;
		((ThreadXsensRead*)XsensRead->threadType_)->sensorsSxens[3].required = true;

		using namespace std;
		using namespace Eigen;

		if (XsensRead->toRUNfromGUI == false)
			throw std::runtime_error("Precisa iniciar o thread XsensRead");

		q0 = 0;
		q1 = 0;
		q2 = 0;
		q3 = 0;
		imus_data[18] = { 0 };
		states_data[10] = { 0 };
		rad2deg = 180 / M_PI;
		offCount = 0;
		euler_k[3] = { 0 };
		omega_k[3] = { 0 };
		Ts = _Ts;

		qASGD1_qk = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
		qASGD2_qk = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
		qASGD3_qk = Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
		qASGD1_Pk = Matrix4f::Identity();
		qASGD2_Pk = Matrix4f::Identity();
		qASGD3_Pk = Matrix4f::Identity();

		Q1 = Matrix4f::Identity() * 5.476e-6; // Usar Eq. 19...
		Q2 = Q1;
		Q3 = Q1;

		gyro1 = Vector3f(0.0f, 0.0f, 0.0f);
		gyro2 = Vector3f(0.0f, 0.0f, 0.0f);
		gyro3 = Vector3f(0.0f, 0.0f, 0.0f);
		gyro4 = Vector3f(0.0f, 0.0f, 0.0f);
		acc1 = Vector3f(0.0f, 0.0f, 0.0f);
		acc2 = Vector3f(0.0f, 0.0f, 0.0f);
		acc3 = Vector3f(0.0f, 0.0f, 0.0f);
		acc4 = Vector3f(0.0f, 0.0f, 0.0f);
		omg_norm = gyro1.norm();
		mi = 0;

		{
			std::unique_lock<std::mutex> _(_mtx);
			pw.SCOPE.y_axis.minimum = -3.14f;
			pw.SCOPE.y_axis.maximum = 3.14f;
			pw.SCOPE.x_axis.minimum = 0.0f;
			pw.SCOPE.x_axis.maximum = T_exec;
			pw.clearItems();
			pw.addItem("omega_p");
			pw.addItem("theta_p");
			pw.addItem("alpha_p");
		}
	}

	void _firstLoop() {

	}

	void _cleanup() {

	}
	fromXsensRead imu_shared_data;
	void _loop() {

		using namespace std;
		using namespace Eigen;

		XsensRead->getData(&imu_shared_data);
		memcpy(imus_data, imu_shared_data.imus_data, sizeof(imus_data));


		gyro1 << imus_data[0], imus_data[1], imus_data[2];
		acc1 << imus_data[3], imus_data[4], imus_data[5];
		gyro2 << imus_data[6], imus_data[7], imus_data[8];
		acc2 << imus_data[9], imus_data[10], imus_data[11];
		gyro3 << imus_data[12], imus_data[13], imus_data[14];
		acc3 << imus_data[15], imus_data[16], imus_data[17];
		gyro4 << imus_data[18], imus_data[19], imus_data[20];
		acc4 << imus_data[21], imus_data[22], imus_data[24];

		q0 = qASGD1_qk(0);
		q1 = qASGD1_qk(1);
		q2 = qASGD1_qk(2);
		q3 = qASGD1_qk(3);

		Zc << 2 * (q1 * q3 - q0 * q2),
			2 * (q2 * q3 + q0 * q1),
			(q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
		F_obj = Zc - acc1.normalized(); // Eq.23

		Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
			2 * q1, 2 * q0, 2 * q3, 2 * q2,
			2 * q0, -2 * q1, -2 * q2, 2 * q3;

		GradF = Jq.transpose() * F_obj; // Eq.25

		omg_norm = gyro1.norm();
		mi = mi0 + Beta * Ts * omg_norm; // Eq.29

		z_k = qASGD1_qk - mi * GradF.normalized(); // Eq.24
		z_k.normalize();

		OmG << 0, -gyro1(0), -gyro1(1), -gyro1(2),
			gyro1(0), 0, gyro1(2), -gyro1(1),
			gyro1(1), -gyro1(2), 0, gyro1(0),
			gyro1(2), gyro1(1), -gyro1(0), 0;
		OmG = 0.5 * OmG;

		Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

		// Process noise covariance update (Eq. 19):
		Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
		Q1 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
		// Projection:
		qASGD1_qk = Psi * qASGD1_qk;
		qASGD1_Pk = Psi * qASGD1_Pk * Psi.transpose() + Q1;
		// Kalman Gain (H is Identity)
		Covar1 = FullPivLU<Matrix4f>(qASGD1_Pk + R);
		if (Covar1.isInvertible())
			Kg1 = qASGD1_Pk * Covar1.inverse();
		// Update (H is Identity)
		qASGD1_qk = qASGD1_qk + Kg1 * (z_k - qASGD1_qk);
		qASGD1_Pk = (Matrix4f::Identity() - Kg1) * qASGD1_Pk;
		qASGD1_qk.normalize();

		// Rotate the quaternion by a quaternion with -(yaw):
		removeYaw(&qASGD1_qk);


		q0 = qASGD2_qk(0);
		q1 = qASGD2_qk(1);
		q2 = qASGD2_qk(2);
		q3 = qASGD2_qk(3);

		Zc << 2 * (q1 * q3 - q0 * q2),
			2 * (q2 * q3 + q0 * q1),
			(q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
		F_obj = Zc - acc2.normalized(); // Eq.23

		Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
			2 * q1, 2 * q0, 2 * q3, 2 * q2,
			2 * q0, -2 * q1, -2 * q2, 2 * q3;

		GradF = Jq.transpose() * F_obj; // Eq.25

		omg_norm = gyro2.norm();
		mi = mi0 + Beta * Ts * omg_norm; // Eq.29

		z_k = qASGD2_qk - mi * GradF.normalized(); // Eq.24
		z_k.normalize();

		OmG << 0, -gyro2(0), -gyro2(1), -gyro2(2),
			gyro2(0), 0, gyro2(2), -gyro2(1),
			gyro2(1), -gyro2(2), 0, gyro1(0),
			gyro2(2), gyro2(1), -gyro2(0), 0;
		OmG = 0.5 * OmG;

		Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

		// Process noise covariance update (Eq. 19):
		Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
		Q2 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
		// Projection:
		qASGD2_qk = Psi * qASGD2_qk;
		qASGD2_Pk = Psi * qASGD2_Pk * Psi.transpose() + Q2;
		// Kalman Gain (H is Identity)
		Covar2 = FullPivLU<Matrix4f>(qASGD2_Pk + R);
		if (Covar2.isInvertible())
			Kg2 = qASGD2_Pk * Covar2.inverse();
		// Update (H is Identity)
		qASGD2_qk = qASGD2_qk + Kg2 * (z_k - qASGD2_qk);
		qASGD2_Pk = (Matrix4f::Identity() - Kg2) * qASGD2_Pk;
		qASGD2_qk.normalize();

		// Rotate the quaternion by a quaternion with -(yaw):
		removeYaw(&qASGD2_qk);


		// Foot:
		q0 = qASGD3_qk(0);
		q1 = qASGD3_qk(1);
		q2 = qASGD3_qk(2);
		q3 = qASGD3_qk(3);

		Zc << 2 * (q1 * q3 - q0 * q2),
			2 * (q2 * q3 + q0 * q1),
			(q0 * q0 - q1 * q1 - q2 * q2 - q3 * q3);
		F_obj = Zc - acc3.normalized(); // Eq.23

		Jq << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
			2 * q1, 2 * q0, 2 * q3, 2 * q2,
			2 * q0, -2 * q1, -2 * q2, 2 * q3;

		GradF = Jq.transpose() * F_obj; // Eq.25

		omg_norm = gyro3.norm();
		mi = mi0 + Beta * Ts * omg_norm; // Eq.29

		z_k = qASGD3_qk - mi * GradF.normalized(); // Eq.24
		z_k.normalize();

		OmG << 0, -gyro3(0), -gyro3(1), -gyro3(2),
			gyro3(0), 0, gyro3(2), -gyro3(1),
			gyro3(1), -gyro3(2), 0, gyro3(0),
			gyro3(2), gyro3(1), -gyro3(0), 0;
		OmG = 0.5 * OmG;

		Psi = (1 - ((omg_norm * Ts) * (omg_norm * Ts)) / 8) * Matrix4f::Identity() + 0.5 * Ts * OmG;

		// Process noise covariance update (Eq. 19):
		Xi << q0, q3, -q2, -q3, q0, q1, q2, -q1, q0, -q1, -q2, -q3;
		Q3 = 0.5 * Ts * Xi * (Matrix3f::Identity() * 5.476e-6) * Xi.transpose();
		// Projection:
		qASGD3_qk = Psi * qASGD3_qk;
		qASGD3_Pk = Psi * qASGD3_Pk * Psi.transpose() + Q3;
		// Kalman Gain (H is Identity)
		Covar3 = FullPivLU<Matrix4f>(qASGD3_Pk + R);
		if (Covar3.isInvertible())
			Kg3 = qASGD3_Pk * Covar3.inverse();
		// Update (H is Identity)
		qASGD3_qk = qASGD3_qk + Kg3 * (z_k - qASGD3_qk);
		qASGD3_Pk = (Matrix4f::Identity() - Kg3) * qASGD3_Pk;
		qASGD3_qk.normalize();

		// Rotate the quaternion by a quaternion with -(yaw):
		removeYaw(&qASGD3_qk);

		// Euler angles between IMUs:
		Vector3f knee_euler = quatDelta2Euler(qASGD2_qk, qASGD1_qk);
		Vector3f ankle_euler = quatDelta2Euler(qASGD3_qk, qASGD2_qk);


		// Remove arbitrary IMU attitude:
		if (float(offCount * _Ts) < 2) // 2 segundos
		{
			q12Off.w() += (_Ts / 1) * qDelta(qASGD2_qk, qASGD1_qk)(0);
			q12Off.x() += (_Ts / 1) * qDelta(qASGD2_qk, qASGD1_qk)(1);
			q12Off.y() += (_Ts / 1) * qDelta(qASGD2_qk, qASGD1_qk)(2);
			q12Off.z() += (_Ts / 1) * qDelta(qASGD2_qk, qASGD1_qk)(3);

			q23Off.w() += (_Ts / 1) * qDelta(qASGD3_qk, qASGD2_qk)(0);
			q23Off.x() += (_Ts / 1) * qDelta(qASGD3_qk, qASGD2_qk)(1);
			q23Off.y() += (_Ts / 1) * qDelta(qASGD3_qk, qASGD2_qk)(2);
			q23Off.z() += (_Ts / 1) * qDelta(qASGD3_qk, qASGD2_qk)(3);
			offCount++;
		}
		else
		{
			q12Off.normalize();
			q23Off.normalize();
			Vector4f q12(q12Off.w(), q12Off.x(), q12Off.y(), q12Off.z());
			Vector4f q23(q23Off.w(), q23Off.x(), q23Off.y(), q23Off.z());
			knee_euler = quatDelta2Euler(qDelta(qASGD2_qk, q12), qASGD1_qk);
			ankle_euler = quatDelta2Euler(qDelta(qASGD3_qk, q23), qASGD2_qk);
		}

		// Relative Angular Velocity
		Vector3f knee_omega = RelVector(qDelta(qASGD1_qk, qASGD2_qk), gyro1, gyro2);
		Vector3f ankle_omega = RelVector(qDelta(qASGD2_qk, qASGD3_qk), gyro2, gyro3);

		// Relative Acc (Linear) in IMU1 frame:
		Vector3f left_shank2thigh_acc = RelVector(qDelta(qASGD1_qk, qASGD2_qk), acc1, acc2);
		Vector3f left_foot2shank_acc = RelVector(qDelta(qASGD2_qk, qASGD3_qk), acc2, acc3);

		// IMU2 Relative Acc (Angular) in IMU2 frame:
		Vector3f left_shank2thigh_alpha = RelAngAcc(qDelta(qASGD2_qk, qASGD1_qk), \
			knee_omega, left_shank2thigh_acc);

		Vector3f s2tL_alpha = Quaternionf(qASGD1_qk).toRotationMatrix() * left_shank2thigh_alpha;

		// Finite Difference Acc approximation:

		euler_k[0] = knee_euler(0);
		float acc_euler = (euler_k[0] - 2 * euler_k[1] + euler_k[2]) / (Ts * Ts);
		euler_k[2] = euler_k[1];
		euler_k[1] = euler_k[0];


		omega_k[0] = knee_omega(0);
		float knee_acc_omega = (3 * omega_k[0] - 4 * omega_k[1] + omega_k[2]) / (2 * Ts);
		omega_k[2] = omega_k[1];
		omega_k[1] = omega_k[0];

		if (float(offCount * _Ts) < 2) // 2 segundos
		{
			states_data[0] = 0;
			states_data[1] = 0;
			states_data[2] = 0;
			states_data[3] = 0;
			states_data[4] = 0;
			states_data[5] = 0;
			states_data[6] = 0;
			states_data[7] = 0;
			states_data[8] = 0;
			states_data[9] = 0;

		}
		else {
			states_data[0] = knee_euler(0);  // knee_pos
			states_data[1] = knee_omega(1);  // knee_vel
			states_data[2] = knee_acc_omega; // knee_acc
			states_data[3] = -ankle_euler(0); // ankle_pos
			states_data[4] = -ankle_omega(0); // ankle_vel
			states_data[5] = 0;				 // ankle_acc
			states_data[6] = 0;				 // Exo pos
			states_data[7] = gyro4(0);		 // Exo vel
			states_data[8] = 0;				 // Exo acc
			states_data[9] = 0;
		}

		{ // sessao critica
			unique_lock<mutex> _(_mtx);
			memcpy(data.data, states_data, sizeof(states_data));
		} // fim da sessao critica

		_datalog[time_index][0] = timer->get_current_time_f();
		for (int i = 0; i < 10; i++) {
			_datalog[time_index][i + 1] = states_data[i];
		}

		if (true) {
			{
				std::unique_lock<std::mutex> _(_mtx);
				pw.items[0].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][1]));
				pw.items[1].data.push_back(ImVec2((float)_datalog[time_index][0], (float)_datalog[time_index][4]));
			}
		}
	}

	bool cbPlot = false;
	void _updateGUI() {
		ImGui::Begin(_name.c_str());

		ImGui::Checkbox("Graph", &cbPlot);
		plotting = cbPlot;
		if (plotting) {
			{
				std::unique_lock<std::mutex> _(_mtx);
				pw.showNewWindow();
			}
		}

		ImGui::End();
	}

	void getData(void* _data) {
		_mtx.lock();
		memcpy(_data, &data, sizeof(fromXsensLeo));
		_mtx.unlock();
	}

private:

	void removeYaw(Eigen::Vector4f* quat)
	{
		float q0 = (*quat)(0);
		float q1 = (*quat)(1);
		float q2 = (*quat)(2);
		float q3 = (*quat)(3);
		float yaw = atan2f(2 * q1 * q2 + 2 * q0 * q3, \
			q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2);
		Eigen::Matrix4f  Qy = Eigen::Matrix4f::Identity() * cosf(-yaw / 2);
		Qy(0, 3) = -sinf(-yaw / 2);
		Qy(1, 2) = Qy(0, 3);
		Qy(2, 1) = -Qy(0, 3);
		Qy(3, 0) = -Qy(0, 3);
		*quat = Qy * (*quat);
		quat->normalize();
	}

	Eigen::Vector4f qDelta(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m)
	{
		float qr0 = quat_r(0);
		float qr1 = quat_r(1);
		float qr2 = quat_r(2);
		float qr3 = quat_r(3);
		// q_m conjugate (*q_m):
		float qm0 = quat_m(0);
		float qm1 = -quat_m(1);
		float qm2 = -quat_m(2);
		float qm3 = -quat_m(3);

		Eigen::Vector4f q;
		// quaternion product: q_r x *q_m:
		q(0) = qr0 * qm0 - qr1 * qm1 - qr2 * qm2 - qr3 * qm3;
		q(1) = qr0 * qm1 + qr1 * qm0 + qr2 * qm3 - qr3 * qm2;
		q(2) = qr0 * qm2 - qr1 * qm3 + qr2 * qm0 + qr3 * qm1;
		q(3) = qr0 * qm3 + qr1 * qm2 - qr2 * qm1 + qr3 * qm0;
		return q;
	}

	Eigen::Vector3f quatDelta2Euler(const Eigen::Vector4f quat_r, const Eigen::Vector4f quat_m)
	{
		using namespace Eigen;
		// quaternion product: q_r x *q_m:
		Vector4f q = qDelta(quat_r, quat_m);
		Vector3f euler;
		euler(0) = atan2f(2 * q(2) * q(3) + 2 * q(0) * q(1), q(3) * q(3) - q(2) * q(2) - q(1) * q(1) + q(0) * q(0));
		euler(1) = -asinf(2 * q(1) * q(3) - 2 * q(0) * q(2));
		euler(2) = atan2f(2 * q(1) * q(2) + 2 * q(0) * q(3), q(1) * q(1) + q(0) * q(0) - q(3) * q(3) - q(2) * q(2));
		return euler;
	}

	Eigen::Vector3f RelVector(const Eigen::Vector4f rel_quat, const Eigen::Vector3f vec_r, const Eigen::Vector3f vec_m)
	{
		return  (vec_m - Eigen::Quaternionf(rel_quat).toRotationMatrix() * vec_r);
	}

	Eigen::Vector3f RelAngAcc(const Eigen::Vector4f rel_quat, const Eigen::Vector3f rel_ang_vel, const Eigen::Vector3f rel_linear_acc)
	{
		using namespace Eigen;
		Matrix3f Rot = Quaternionf(rel_quat).toRotationMatrix();

		Vector3f linAccFrame2 = Rot.transpose() * rel_linear_acc;
		Vector3f angVelFrame2 = Rot.transpose() * rel_ang_vel;

		float omg_x = angVelFrame2(0);
		float omg_y = angVelFrame2(1);
		float omg_z = angVelFrame2(2);
		float acc_x = linAccFrame2(0);
		float acc_y = linAccFrame2(1);
		float acc_z = linAccFrame2(2);
		float alpha_x, alpha_y, alpha_z;

		// using centriptal and radial acc decompositon:
		float norm_zy = sqrt(acc_z * acc_z + acc_y * acc_y);
		float phi = atan2f(-acc_y, acc_z); // -y devido a orientacao adotada das imus nas pernas
		alpha_x = norm_zy * sinf(phi); // aproximando R=1

		float norm_xz = sqrt(acc_x * acc_x + acc_z * acc_z);
		phi = atan2f(acc_x, -acc_z);
		alpha_y = norm_xz * sinf(phi); // aproximando R=1

		float norm_xy = sqrt(acc_x * acc_x + acc_y * acc_y);
		phi = atan2f(acc_y, -acc_x);
		alpha_z = norm_xy * sinf(phi); // aproximando R=1

		return Vector3f(alpha_x, alpha_y, alpha_z);
	}

};
