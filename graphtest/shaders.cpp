#include "general.hpp"
#include "sub.hpp"

using evec3 = Eigen::Vector3<halff>;

const extern sindex RAYNUM_LIMIT_BRUNCH;
const extern exindex RAYNUM_LIMIT_ALL;
const extern size_t ATTRIBUTE_SIZE;

using brunch = toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH,toolkit::attributeFramework<ATTRIBUTE_SIZE>>::brunch;

payloadContent MissShader(const closesthit& str, brunch& nextgenlocal, const las* plas, bool& isTerminate) {
	using evec3 = Eigen::Vector3<halff>;
	//using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(0.0, 0.0, -1.0);

	halff doter = direction.normalized().dot(-light.normalized());
	doter = std::max(0.0, doter);
	doter = pow(doter, 0.5);

	//–³ŒÀ‰“‚É”ò‚ñ‚Ås‚Á‚½
	isTerminate = true;

	//doter = 0.0_h;
	halff amb = 0.05;// +0.05_h;//ŠÂ‹«Œõ
	return payloadContent({ 1.0,1.0 ,1.0 }, { 0.,0.,0. });
}
payloadContent HitMirror(const closesthit& att, brunch& nextgenlocal, const las* plas,bool& isTerminate) {
	//using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, (const las*)plas);

	//”½ËŒõ‚ğ“o˜^
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent(hvec3(0.5,0.5,0.5));
}
payloadContent HitRandom(const closesthit& att, brunch& nextgenlocal, const las* plas, bool& isTerminate) {
	//using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, (const las*)plas);

	//”½ËŒõ‚ğ“o˜^
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent(hvec3(0.5,0.5,0.5));
}
payloadContent HitLight(const closesthit& att, brunch& nextgenlocal, const las* plas, bool& isTerminate) {
	//using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, (const las*)plas);
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent({1.0,1.0,1.0}, { std::abs<halff>(attrib.norm.x()),std::abs<halff>(attrib.norm.y()),std::abs<halff>(attrib.norm.z()) });
}
payloadContent HitNormColor(const closesthit& att, brunch& nextgenlocal,const las* plas, bool& isTerminate) {
	//using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, (const las*)plas);

	//”½ËŒõ‚ğ“o˜^
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent({ {1.0,1.0,1.0},hvec3(std::abs<halff>(attrib.norm.x()),std::abs<halff>(attrib.norm.y()) ,std::abs<halff>(attrib.norm.z())) });
}
payloadContent HitLightNorm(const closesthit& att, brunch& nextgenlocal, const las* plas, bool& isTerminate) {
	//using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, (const las*)plas);
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent({ 1.0,1.0,1.0 }, { -attrib.norm.x(),attrib.norm.y(),attrib.norm.z() });
}

template<size_t RAYNUM_LIMIT_ALL, size_t RAYNUM_LIMIT_BRUNCH,size_t ATTRIBUTE_SIZE>typename toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH, toolkit::attributeFramework<ATTRIBUTE_SIZE>>::shader GetColorShader(const hvec3 color) {
	typename toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH, toolkit::attributeFramework<ATTRIBUTE_SIZE>>::shader func = [color](const closesthit& att, brunch& nextgenlocal, const las* plas, bool& isTerminate) {
		using evec3 = Eigen::Vector3<halff>;

		auto attrib = Attrib(att, (const las*)plas);
		//nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
		isTerminate = true;
		
		evec3 ray(0, 0, -1);
		evec3 norm(attrib.norm.x(), attrib.norm.y(), attrib.norm.z());
		double rr=fabs(ray.dot(norm));
		return payloadContent({ 1.0,1.0,1.0 }, { color.x() * rr,color.y() *rr,color.z() *rr });
	};

	return func;
}
payloadContent HitColor(const closesthit& att, brunch& nextgenlocal, const las* plas, bool& isTerminate) {
	//using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, (const las*)plas);

	//”½ËŒõ‚ğ“o˜^
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent(hvec3(std::abs<halff>(attrib.norm.x()), std::abs<halff>(attrib.norm.y()), std::abs<halff>(attrib.norm.z())));
}