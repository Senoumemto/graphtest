#include "general.hpp"
#include "sub.hpp"

using evec3 = Eigen::Vector3<halff>;

const extern sindex RAYNUM_LIMIT_BRUNCH;
const extern exindex RAYNUM_LIMIT_ALL;

using brunch = toolkit::materialer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH>::brunch;

payloadContent MissShader(const closesthit& str, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;

	evec3 direction(str.r.way().data());
	evec3 light(0.0_h, -1.0_h, 0.0_h);

	halff doter = direction.normalized().dot(-light.normalized());
	doter = std::max(0.0_h, doter);
	doter = pow(doter, 1.0_h);

	//–³ŒÀ‰“‚É”ò‚ñ‚Ås‚Á‚½
	isTerminate = true;

	//doter = 0.0_h;
	halff amb = +0.05_h;//ŠÂ‹«Œõ
	return payloadContent({ 1.0_h,1.0_h ,1.0_h }, { doter + amb,doter + amb,0.0_h });
}
payloadContent HitMirror(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas,bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, ptlas);

	//”½ËŒõ‚ğ“o˜^
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent({ 0.0_h,0.5_h,0.5_h });
}
payloadContent HitLight(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, ptlas);
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent({1.0_h,1.0_h,1.0_h}, { std::abs<halff>(attrib.norm.x()),std::abs<halff>(attrib.norm.y()),std::abs<halff>(attrib.norm.z()) });
}
payloadContent HitColor(const closesthit& att, brunch& nextgenlocal, sptr<tlas> ptlas, bool& isTerminate) {
	using namespace half_float::literal;
	using evec3 = Eigen::Vector3<halff>;

	auto attrib = Attrib(att, ptlas);

	//”½ËŒõ‚ğ“o˜^
	nextgenlocal.push_head(parentedRay({ att.r.index(), attrib.refrect }));
	isTerminate = false;

	return payloadContent(hvec3::Make(attrib.norm));
}