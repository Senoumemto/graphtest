#include "general.hpp"

using evec3 = Eigen::Vector3<halff>;

const extern sindex RAYNUM_LIMIT_BRUNCH;
const extern exindex RAYNUM_LIMIT_ALL;

using brunch = toolkit::materialer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH>::brunch;

payloadContent HitMirror(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//�@�������߂�
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//�q�b�g�_�����߂�
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//���ʔ���
	auto a = (-evec3(att.r.way().data())).dot(norm);
	//evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);
	evec3 refrectway = (2.0_h * ((-evec3(att.r.way().data())).dot(norm)) * norm + evec3(att.r.way().data())).normalized();

	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });
	nextgenlocal.push_head(parentedRay({ att.r.index(), ref }));

	//std::cout << norm.norm().operator float() << std::endl;
	if (norm.z() <= 0.0001_h)
		int a = 0;

	//���˂���̂�term�łȂ�
	isTerminate = false;
	return payloadContent({ 0.5_h,0.5_h,0.5_h });
	//return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { std::abs<halff>(refrectway.x()), std::abs<halff>(refrectway.y()), std::abs<halff>(refrectway.z()) });
	//return payloadContent({std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z())});
	//return payloadContent({ std::abs<halff>(refrectway.x()),std::abs<halff>(refrectway.y()),std::abs<halff>(refrectway.z()) });
}
payloadContent HitLight(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//�@�������߂�
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//�q�b�g�_�����߂�
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//���ʔ���
	auto a = (-evec3(att.r.way().data())).dot(norm);
	//evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);
	evec3 refrectway = (2.0_h * ((-evec3(att.r.way().data())).dot(norm)) * norm + evec3(att.r.way().data())).normalized();

	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });
	nextgenlocal.push_head(parentedRay({ att.r.index(), ref }));

	//std::cout << norm.norm().operator float() << std::endl;
	if (norm.z() <= 0.0001_h)
		int a = 0;

	//���˂���̂�term�łȂ�
	isTerminate = false;
	//return payloadContent({ 0.5_h,0.5_h,0.5_h });
	halff xxx = norm.y() < -0.001 ? 1.0_h : 0.0_h;
	//return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { 1.0_h,xxx,xxx });
	return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z()) });
}

payloadContent MissShader(const closesthit& str, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(0.0_h, -1.0_h, 0.0_h);

	halff doter = direction.normalized().dot(-light.normalized());
	doter = std::max(0.0_h, doter);
	doter = pow(doter, 1.0_h);

	//�������ɔ��ōs����
	isTerminate = true;

	doter = 0.0_h;
	halff amb = +0.00_h;//����
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { doter + amb,doter + amb,doter + amb });
}

payloadContent HitPos(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//�@�������߂�
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//�q�b�g�_�����߂�
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//���ʔ���
	auto a = (-evec3(att.r.way().data())).dot(norm);
	evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);


	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });
	nextgenlocal.push_head(parentedRay({ att.r.index(), ref }));

	
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { 0.0_h,hitpoint.y() / 20.0_h ,-hitpoint.z() / 20.0_h });
	//return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z()) });
}

payloadContent HitWay(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	//�@�������߂�
	auto tri = ptlas->at(att.tri.blasId()).second->triangles.at(att.tri.triId());
	evec3 norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	//�q�b�g�_�����߂�
	evec3 hitpoint = (evec3(att.r.way().data()) * att.uvt.at(2)) + evec3(att.r.org().data());

	//���ʔ���
	auto a = (-evec3(att.r.way().data())).dot(norm);
	//evec3 refrectway = (norm * a * 2.0_h - evec3(att.r.way().data())).normalized();// evec3(att.r.way().data()) + norm * (a * -2.0_h);
	evec3 refrectway = (2.0_h * ((-evec3(att.r.way().data())).dot(norm)) * norm + evec3(att.r.way().data())).normalized();


	ray ref;
	ref.way() = hvec3({ refrectway.x(),refrectway.y(),refrectway.z() });
	ref.org() = hvec3({ hitpoint.x(),hitpoint.y(),hitpoint.z() });

	isTerminate = true;
	//return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { 1.0_h,1.0_h,1.0_h });
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { ref.way().x() ,ref.way().y() ,ref.way().z() });
	//return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(norm.x()),std::abs<halff>(norm.y()),std::abs<halff>(norm.z()) });
}