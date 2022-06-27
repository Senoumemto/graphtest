#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <half.hpp>
#include <array>
#include <memory>
#include <algorithm>
#include <deque>
#include <optional>
#include <functional>
#include <tuple>
#include <numbers>
#include <random>

#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>

#include "int24.hpp"


using word = int16_t;
using uword = uint16_t;
using halff = half_float::half;
using hmat4 = Eigen::Matrix4<halff>;
using sindex = uint16_t;
using exindex = uint32_t;

class exindicesWithHead :public std::vector<exindex> {
	
public:
	using super = std::vector<exindex>;
	size_t head;

	exindicesWithHead(const super& s) :super(s) { head = 0; }
	exindicesWithHead(size_t size) :super(size) { head = 0; }
	exindicesWithHead() :super() { head = 0; }

	exindicesWithHead& push_head(exindex i) {
		this->at(head) = i;

		head++;

		return *this;
	}
};

template<typename T>using uptr = std::unique_ptr<T>;
template<typename T>using sptr = std::shared_ptr<T>;

template<typename T>class vec3 :public std::array<T, 3>{
	using super = std::array<T, 3>;
public:
	vec3() :super() {}
	vec3(const super& s) :super(s) {}

	T& x() { return this->at(0); }
	T& y() { return this->at(1); }
	T& z() { return this->at(2); }

	const T& x() const { return this->at(0); }
	const T& y() const { return this->at(1); }
	const T& z() const { return this->at(2); }

	static vec3<T> Make(const Eigen::Vector3<T>& v) {
		vec3<T> ret;

		for (size_t i = 0; i < 3; i++)
			ret.at(i) = v[i];

		return ret;
	}
	static vec3<T> Make(const Eigen::Vector4<T>& v,bool divByW) {
		vec3<T> ret;

		for (size_t i = 0; i < 3; i++)
			ret.at(i) = v[i] / (divByW ? v[3] : (T)1.0);

		return ret;
	}

	static vec3<T> Zero() {
		return vec3<T>({T(0.0),T(0.0),T(0.0)});
	}
};
using hvec3 = vec3<halff>;

template<typename T>using tri = vec3<vec3<T>>;
template<typename T>using mod = std::vector<tri<T>>;

using htri = tri<halff>;

using dmod = mod<double>;
using hmod = mod<halff>;

class aabb :public std::pair<hvec3, hvec3> {
	using super = std::pair<hvec3, hvec3>;
public:
	aabb(const super&s):super(s){}
	aabb():super(){}

	hvec3& min() { return this->first; }
	hvec3& max() { return this->second; }
	const hvec3& min()const { return this->first; }
	const hvec3& max()const { return this->second; }
};
//blas�c���[���\������m�[�h �߂��t�̂ǂ��炩����ʂ���t���O�������Ă���
struct blasnode {
	aabb data;
	bool isVertex;//�߃m�[�h�ł���
};
using nodetree = std::vector<blasnode>;

class blas{
public:
	nodetree tree;//�v�f��1�Ԃ���n�܂�n�ԃm�[�h�̍��q�m�[�h��2n �E�q�m�[�h��2n+1�ł���
	hmod triangles;
	
	aabb MakeAABB(const hmod::iterator& b, const hmod::iterator& e);
	void innerSort(hmod::iterator b, hmod::iterator e,int axis);
public:
	blas(const dmod& d);
};
//����
class ray :public std::pair<hvec3, hvec3> {
	using super = std::pair<hvec3, hvec3>;
protected:
	std::optional<exindex> id;//rooter�ɓo�^�����܂ł�id�Ȃ�
public:
	hvec3& org();
	hvec3& way();

	const hvec3& org()const;
	const hvec3& way()const;

	ray(const super& s) :super(s) {}
	ray() {}

	ray& indexed(exindex _id) { id = _id; return *this; }
	exindex index()const { return id.value(); }
};
using rays = std::vector<ray>;
//�e�������C �V�F�[�_�[����rooter�܂ł�nextgen�̕\���Ɏg���Ă�
class parentedRay :private std::pair<exindex, ray> {
	using super = std::pair<exindex, ray>;
public:
	parentedRay():super(){}
	parentedRay(const super& s) :super(s) {}
	parentedRay(const ray& r) :super(std::numeric_limits<exindex>::max(), r) {}

	exindex& index() { return this->first; }
	ray& GetRay() { return this->second; }
	const exindex& index() const{ return this->first; }
	const ray& GetRay() const{ return this->second; }
};
using parentedRays = std::vector<parentedRay>;

//�J����
class camera : public rays {
public:
	//1x1�̉𑜓xh*v�̃X�N���[�����œ_����dist�����������J�������쐬����
	camera(size_t h, size_t v, double dist, double asp = 1.0);

	static halff CalcDistFromFov(const halff& fov);
};

//blas�Ƃ��̕ϊ���o�^����\���@scene�ɑΉ�����
class las :public std::vector<std::pair<hmat4, sptr<blas>>>{
	using blasset = std::pair<hmat4, sptr<blas>>;
	using super = std::vector<blasset>;
public:
	las(const super&s):super(s){}
	las():super(){}
};

//g-index tlas����tri�����ǂ��index
class gindex :public std::pair<sindex, sindex> {
	using super = std::pair<sindex, sindex>;
public:
	gindex():super(){}
	gindex(const super&s):super(s){}

	sindex& blasId() { return this->first; }
	sindex& triId() { return this->second; }
	const sindex& blasId() const{ return this->first; }
	const sindex& triId() const{ return this->second; }
};
using broadphaseResultElement = std::pair<ray, gindex>;
using broadphaseResults = std::deque<broadphaseResultElement>;

//uvt��intoExtend,vsTriangle�̕Ԃ�l
class vsTriangleResult :private std::pair<hvec3, bool> {
	using super = std::pair<hvec3, bool>;
public:
	vsTriangleResult(const super&s):super(s){}
	vsTriangleResult(){}

	hvec3& uvt() { return this->first; }
	bool& intoExtend() { return this->second; }
	const hvec3& uvt() const{ return this->first; }
	const bool& intoExtend() const{ return this->second; }


};
struct narrowphaseResultElement {
	ray r;
	gindex tri;
	vsTriangleResult vsTriRez;

	narrowphaseResultElement(){}
	narrowphaseResultElement(const broadphaseResultElement& bprez, const vsTriangleResult& rez) :r(bprez.first), tri(bprez.second), vsTriRez(rez){}
};
using narrowphaseResults = std::deque<narrowphaseResultElement>;

using closesthit = narrowphaseResultElement;

using closesthits = std::vector<closesthit>;

//���C�̑����Ƃ��Đe�ƃR���e���c������
template<typename contentType>class payloadbase {

	contentType content;

public:
	virtual payloadbase& InstallContent(const contentType& c) {
		this->content = c;

		return *this;
	}
	const contentType& GetContent()const { return content; }

	exindex parent;//�e������exindex(�S����)


};

class payloadContent :private std::pair<hvec3, hvec3> {
	using super = std::pair<hvec3, hvec3>;
	
public:
	hvec3& scale() { return this->first; }//�F��Ԃ̊g�k(����)
	hvec3& translate() { return this->second; }//�F�̕��i(����)

	const hvec3& scale() const{ return this->first; }//�F��Ԃ̊g�k(����)
	const hvec3& translate() const{ return this->second; }//�F�̕��i(����)

	payloadContent(const hvec3& scale, const hvec3& trans,bool allowNegativeValue) { 
		if (allowNegativeValue) {
			this->scale() = scale;
			this->translate() = trans;
		}
		else {
			hvec3 junk;
			for (size_t i = 0; i < 3; i++)
				junk.at(i) = std::abs<halff>(scale.at(i));
			this->scale() = junk;

			for (size_t i = 0; i < 3; i++)
				junk.at(i) = std::abs<halff>(trans.at(i));
			this->translate() = junk;
		}

	}
	payloadContent(const std::array<halff,3>& scale, const std::array<halff, 3>& trans, bool allowNegativeValue=false):payloadContent(hvec3(scale), hvec3(trans), allowNegativeValue) {}
	payloadContent(const hvec3& scale, bool allowNegativeValue = false):payloadContent(scale, hvec3::Zero(), allowNegativeValue) {}
	payloadContent(const std::array<halff,3>& scale, bool allowNegativeValue = false):payloadContent(hvec3(scale), allowNegativeValue) {}
	payloadContent(const super&s):super(s){}
	payloadContent(const payloadContent&c, bool allowNegativeValue = false):payloadContent(c.scale(),c.translate(), allowNegativeValue){}
	payloadContent(){}
};
using payload = payloadbase<payloadContent>;
using payloads = std::vector<payload>;

template<typename paytype>using payed_ray = std::pair<ray, paytype>;
template<typename paytype>using payed_rays = std::vector<payed_ray<paytype>>;

//bitmap��\��
template<size_t res>struct bitmap :public std::array<hvec3, res* res>{};


//�O���{�̋@�\������
namespace toolkit {
	//�������R���N�V����
	template<size_t ATTRIB_SIZE>using attributeFramework = std::array<halff, ATTRIB_SIZE>;
	template<typename attribute>using triangleAttributeFramework = std::array<attribute, 3>;
	template<typename attribute>using triangleAttributesFramework = std::vector<triangleAttributeFramework<attribute>>;
	template<typename attribute>using attributesSetFramework = std::vector<triangleAttributesFramework<attribute>>;
	template<typename payload_type,size_t RAYNUM_LIMIT_ALLGEN,size_t RAYNUM_LIMIT_GENERATION,size_t RAYNUM_LIMIT_TERMINATES,size_t ATTRIB_SIZE,size_t BLASNUM_LIMIT,size_t TRIANGLES_NUM_LIMIT>class memoryCollection {
		using payloads = std::vector<payload_type>;
		using attribute = attributeFramework<ATTRIB_SIZE>;
		using triangleAttribute = triangleAttributeFramework<attribute>;
		using triangleAttributes = triangleAttributesFramework<attribute>;
		using attributesSet = attributesSetFramework<attribute>;//blasid-attributes�̃��X�g

		uptr<rays> allows_allgen;//�S�����ray���X�g(�O���[�o�����)
		uptr<payloads>payloads_allgen;//�S����̃y�C���[�h���X�g

		uptr<closesthits>closesthits_gen;//�������closesthit

		uptr<exindicesWithHead> terminates_allgen;//�I�[�ɂȂ郌�C(�܂���̔����_)�̏W��

		uptr<attributesSet> attributes;//�ʑ���

		uptr<las>plas;//las
	public:
		memoryCollection() {
			allows_allgen.reset(new rays(RAYNUM_LIMIT_ALLGEN));
			payloads_allgen.reset(new payloads(RAYNUM_LIMIT_ALLGEN));
			closesthits_gen.reset(new closesthits(RAYNUM_LIMIT_GENERATION));
			terminates_allgen.reset(new exindicesWithHead(RAYNUM_LIMIT_TERMINATES));
			attributes.reset(new attributesSet(BLASNUM_LIMIT));
			for (auto& i : attributes.operator*())
				i.resize(TRIANGLES_NUM_LIMIT);

			plas.reset(new las);
		}

		rays* GetAllowsAllgen() { return allows_allgen.get(); }
		payloads* GetPayloadsAllgen() { return payloads_allgen.get(); }
		closesthits* GetClosesthitsGen() { return closesthits_gen.get(); }
		exindicesWithHead* GetTerminatesGen() { return terminates_allgen.get(); }
		las* GetLAS() { return plas.get(); }
		attributesSet* GetAttributes() { return attributes.get(); }
	};


	//���C��z�z���Ă������
	template<size_t cachesize,typename paytype>class generator {
		using payloads = std::vector<paytype>;

		//sptr<payed_rays<paytype>> cache;
		//�������������ɐڑ�����

		exindex MakeUniqueRayId(){
			static exindex id = 0;
			return id++;
		}
		exindex nowhead;//�ǉ�����郌�C�̓��ʒu
		exindex genhead;//������̓��ʒu
	public:
		generator():nowhead(0),genhead(0) {}

		//���C�W����o�^����@�e�̂��Ȃ�(0����̃��C��ǉ�)=�J����
		void RegisterRays(const rays& rs, rays* rays_allgen, payloads* pays_allgen) {
			for (exindex i = 0; i < rs.size(); i++) {
				rays_allgen->at(nowhead + i) = rs.at(i);
				rays_allgen->at(nowhead + i).indexed(nowhead + i);

				pays_allgen->at(nowhead + i).parent = std::numeric_limits<exindex>::max();
			}
			nowhead += rs.size();
		}
		//���C�W����o�^����@�e��=nextgen
		void RegisterRays(const parentedRays& rs, rays* rays_allgen,payloads* pays_allgen) {
			for (exindex i = 0; i < rs.size(); i++) {
				rays_allgen->at(nowhead + i) = rs.at(i).GetRay();
				rays_allgen->at(nowhead + i).indexed(nowhead + i);

				pays_allgen->at(nowhead + i).parent = rs.at(i).index();//�y�C���[�h�ɐe����ǉ�
			}
			nowhead += rs.size();
		}

		rays GetGeneration(exindex& retGenhead, rays* rays_allgen,exindex& generationsize) {
			retGenhead = exindex(genhead);
			generationsize = nowhead - genhead;
			//���㓪���猻�݃w�b�h�܂ł����o��
			rays ret(nowhead - genhead);
			for (int i = genhead; i < nowhead; i++)
				ret.at(i - genhead) = rays_allgen->at(i);
			//= rays(cache->begin() + genhead, cache->begin() + nowhead, [](const std::pair<ray, paytype>& p) {return p.second; });
			//������p�ɓf���o��
			genhead = nowhead;

			return ret;
		}

		//���C�̃T�C�Y��Ԃ�
		exindex GetRaysEnd() {
			return genhead;
		}
	};

	template<size_t coren>class broadphaser {
	protected:

		const halff param_aabbMargin;

		bool vsAABB(const ray& r, const aabb& box, halff tmin = halff(0.0), halff tmax = std::numeric_limits<halff>::infinity()) {

			//std::cout << "AABB" << std::endl;
			//for (const auto& i : box.min())
			//	std::cout << i << "\t";
			//std::cout << std::endl;
			//for (const auto& i : box.max())
			//	std::cout << i << "\t";
			//std::cout << "\n" << std::endl;

			//std::cout << "ray" << std::endl;
			//for (const auto& i : r.way())
			//	std::cout << i << "\t";
			//std::cout << std::endl;
			//for (const auto& i : r.org())
			//	std::cout << i << "\t";
			//std::cout << "\n" << std::endl;


			for (int a = 0; a < 3; a++) {
				halff invD = halff(1.0) / r.way()[a];
				halff t0 = (box.min()[a] - r.org()[a]) * invD;//�ߖ�
				halff t1 = (box.max()[a] - r.org()[a]) * invD;//����
				if (invD < 0.0f)
					std::swap(t0, t1);
				tmin = t0 > tmin ? t0 : tmin;//�ߖʂ̂�����艓������
				tmax = t1 < tmax ? t1 : tmax;//���ʂ̂������߂�����
				//std::cout << tmin << "\t" << tmax << std::endl;

				//if (tmax < tmin)//�������Ă���Ȃ炾��
				if (tmin - tmax > param_aabbMargin)//�������Ă���Ȃ�_���@�܂�tmin�̕����傫����΂��߁@�������}�[�W���L
					return false;
			}
			return true;
		}

		//���C1�ɑ΂��ă��C�g�����s�� ���ʂ̓��C���Ƃ̍L��Փ˃��X�g(gindex)
		std::deque<gindex> Raytrace(const std::array<ray, coren>& rs) {
			std::deque<gindex> ret;
			//�R�A���ƂɃ��C�g�����s�� ���ʂ̓��C���Ƃ̍L��Փ˃��X�g(gindex)
			for (int i = 0; i < coren; i++) {
				auto each = cores.at(i).Raytrace(rs.at(i));
				for (auto& e : each)
					ret.push_back(e);
			}

			return ret;
		}
	public:
		//sptr<tlas>ptlas;//������tlas���C���X�g�[�����Ďg��


		//�g���[�T�[�R�A
		struct core {
			sindex nowi;
			bool isInv;//�������܋t�s�� �ڍs�ł���悤�ɂȂ�܂ŋt�s�𑱂���
			broadphaser<coren>* parent;

			//r�Őe��plas�ɑ΂��ă��C�g���[�V���O���s���L��Փˏ��̃��X�g��Ԃ�
			std::deque<std::pair<ray,gindex>> Raytrace(const ray& r_grobal,const las* plas) {
				using evec3 = Eigen::Vector3<halff>;
				using evec4 = Eigen::Vector4<halff>;
				using namespace half_float::literal;

				std::deque<std::pair<ray,gindex>> ret;
				//blas���Ƃɍs��
				for (sindex blasid = 0; blasid < plas->size();blasid++) {
					const auto& blasset = plas->at(blasid);
					//�ϊ������C�ɓK�p����
					ray r;
					r.indexed(r_grobal.index());//����̃��C�ł���
					r.way() = hvec3::Make(evec4(blasset.first * evec4(r_grobal.way().x(), r_grobal.way().y(), r_grobal.way().z(), 0.0_h)).normalized(), false);
					r.org() = hvec3::Make(evec4(blasset.first * evec4(r_grobal.org().x(), r_grobal.org().y(), r_grobal.org().z(), 1.0_h)), true);

					const sindex leafhead = (blasset.second->tree.size() + 1) / 2;//�t�m�[�h�̈�ԍŏ�

					nowi = 1;//���[�g�ɖ߂�
					isInv = false;
					do {
						//std::cout << "���̃m�[�h\t" << nowi << std::endl;
						//���������t�s���Ȃ�
						if (isInv) {
							//�t�s�����̏����͍��m�[�h�ł��邱��
							if (nowi % 2 == 0) {
								isInv = false;
								nowi = nowi + 1;//����Ɉڍs
							}
							else nowi = (nowi - 1) / 2;//�����łȂ���΋t�s
						}
						else {
							if (parent->vsAABB(r, blasset.second->tree.at(nowi-1).data)) {
								//�q�b�g�Ȃ�
								if (blasset.second->tree.at(nowi-1).isVertex)
									nowi = nowi * 2;//���߂Ȃ珇�s�@2n
								else {
									ret.push_back(std::make_pair(r,std::make_pair(blasid,(sindex)(nowi-leafhead))));//���͗t�Ȃ�\��
									if (nowi % 2) {
										nowi = (nowi - 1) / 2;//�E�Ȃ�t�s
										isInv = true;
									}
									else nowi = nowi + 1;//���Ȃ�ڍs
								}
							}
							else {
								//�q�b�g�łȂ����
								if (nowi % 2 == 0) {
									//�����m�[�h�ł���Έڍs
									nowi = nowi + 1;
								}
								else {
									//���E�m�[�h�ł���΋t�s
									nowi = (nowi - 1) / 2;
									isInv = true;
								}
							}
						}
					} while (nowi != 1);
				}

				return ret;
			}
		};

		std::array<core,coren> cores;

		broadphaser(const halff& aabbMargin):cores(),param_aabbMargin(aabbMargin) {
			for (core& i : cores)
				i.parent = this;
		}

		sptr<broadphaseResults> Broadphase(const rays& rs,const las* plas) {

			sptr<broadphaseResults> ret(new broadphaseResults);

			//rs���畡������r�𓾂�
			for (const ray& r : rs) {
				auto r_sHits = cores.front().Raytrace(r,plas);//�u���[�h�t�F�[�Y���s���L��Փˏ����󂯎��

				for (const auto& g : r_sHits)
					ret->push_back(g);
			}

			return ret;
		}
	};


	class narrowphaser {
	protected:
		std::optional<vsTriangleResult> vsTriangle(const ray& r, const htri& tri,const halff& extendSize);
	public:
		//sptr<tlas>ptlas;//������tlas���C���X�g�[�����Ďg��
		halff param_extendSize;

		sptr<narrowphaseResults> RayTrace(const broadphaseResults& bprez,const halff param_ignoreNearHit, const halff param_ignoreParallelHit, const las* plas);
		narrowphaser(const halff& extendSize):param_extendSize(extendSize){}
	};


	template<exindex cachesize>class obstructer {
	protected:
		closesthits* closests_gen;
	public:
		exindex Anyhit(const narrowphaseResults& nprez,exindex genhead, closesthits* closests_gen) {
			exindex anyhitsize = 0;
			for (const auto& rez : nprez) {
				//closest��extend�̂Ƃ��A����҂�not extend��t�����傫����΍X�V
				//closest��not extend�̂Ƃ��A����҂�not extend�ł���t�����傫�����
				bool tLarger = rez.vsTriRez.uvt().at(2) < closests_gen->at(rez.r.index() - genhead).vsTriRez.uvt().at(2);//����҂�t���傫��


				if (closests_gen->at(rez.r.index() - genhead).vsTriRez.intoExtend() && (!rez.vsTriRez.intoExtend() || tLarger) || !closests_gen->at(rez.r.index() - genhead).vsTriRez.intoExtend() && (!rez.vsTriRez.intoExtend() && tLarger)) {//��̏����ōēo�^
					closests_gen->at(rez.r.index() - genhead) = rez;
					anyhitsize++;
				}
			}

			//for (const auto& c : *cache) {
			//	if (c.uvt.at(2) != std::numeric_limits<halff>::infinity())
			//		std::cout << "index " << c.r.index() << std::endl;
			//}

			return anyhitsize;
		}
		void InstallGeneration(const rays& nowgen,exindex genhead, closesthits* closests_gen) {
			using namespace half_float::literal;
			for (const ray& r : nowgen) {
				closests_gen->at(r.index() - genhead).r = r;//���C��}��
				closests_gen->at(r.index() - genhead).vsTriRez = std::make_pair(hvec3({ 0.0_h, 0.0_h, std::numeric_limits<halff>::infinity() }),true);//�������Ō��� �����Ȃ���Ԃł�extend���󂯓����
			}
		}

		obstructer(){}
	};

	template<exindex cachesize,sindex brunchsize,typename ATTRIB_TYPE>class materializer {


		void ApplyToRay(const hmat4& m, ray& r) {
			using namespace half_float::literal;
			using evec4 = Eigen::Vector4<halff>;
			//r.indexed(r.index());�C���f�b�N�X�͖�������U���ĂȂ�
			r.way() = hvec3::Make(evec4(m * evec4(r.way().x(), r.way().y(), r.way().z(), 0.0_h)).normalized(), false);
			r.org() = hvec3::Make(evec4(m * evec4(r.org().x(), r.org().y(), r.org().z(), 1.0_h)), true);
		}
	public:
		//��̃��C������ꂽ������ �w�b�h�t��
		class brunch : std::array<parentedRay, brunchsize>{
			using super = std::array<parentedRay, brunchsize>;
		public:
			sindex head;

			brunch():super() { head = 0; }
			brunch(const super& s) :super(s) { head = 0; }

			brunch& push_head(const parentedRay& p) {
				super::at(head++) = p;

				return *this;
			}
			parentedRay& at(sindex i) {
				return super::at(i);
			}
			const parentedRay& at(sindex i) const {
				return super::at(i);
			}

		};
		//sptr<tlas>ptlas;
		using shader = std::function<payloadContent(const closesthit&, brunch&,const las*,bool&)>;
		//�V�F�[�_�[���X�g blas-id��p����
		struct shaderlist :public std::vector<shader> {
			shader miss;
		};

		shaderlist mats;//shader HitShader, MissShader;

		const payloads* Shading(const closesthits& hits,parentedRays& nextgen,bool arrowNextgen, payloads* pays_allgen,exindicesWithHead* terminates,size_t anyhitsize,const las* plas,const attributesSetFramework<ATTRIB_TYPE>* attribSet) {
			for (size_t i = 0; i < anyhitsize; i++) {
				const auto r = hits.at(i);
				brunch nextbrunch;//���񐶂��郌�C
				bool isTerminate = false;//���̃��C�͏I�[�ł��邩?
				//�����_�����O�@�Ή������V�F�[�_���Ăяo���ăy�C���[�h���X�g�ɒǉ�
				pays_allgen->at(r.r.index()).InstallContent((r.vsTriRez.uvt().at(2) != std::numeric_limits<halff>::infinity()) ? mats.at(r.tri.blasId()).operator()(r, nextbrunch, plas,isTerminate) : mats.miss(r, nextbrunch, plas,isTerminate));
				
				//�I�[�������オ������Ȃ����
				if (isTerminate || !arrowNextgen)terminates->push_head(r.r.index());

				for (int i = 0; i < nextbrunch.head; i++) {
					ApplyToRay(plas->at(r.tri.blasId()).first.inverse(), nextbrunch.at(i).GetRay());
					nextgen.push_back(nextbrunch.at(i));
				}
			}
			
			return pays_allgen;
		}

		materializer(){}
	};

	template<typename paytype,size_t res> class developer {
		//ee��er���|��������
		void MulHvec(hvec3& ee,const hvec3& er) {
			for (int i = 0; i < 3; i++)
				ee.at(i) *= er.at(i);
		}
		void AddHvec(hvec3& ee, const hvec3& er) {
			for (int i = 0; i < 3; i++)
				ee.at(i) += er.at(i);
		}
		void ClampHvec3(hvec3& ee,const halff& min=halff(0.0), const halff& max=halff(1.0)) {
			using evec3 = Eigen::Vector3<halff>;
			using namespace half_float::literal;

			for (int i = 0; i < 3; i++) {
				if (ee.at(i) > 1.0_h) {
					//std::cout << ee.at(0).operator float() <<"\t"<< ee.at(1).operator float() <<"\t"<< ee.at(2).operator float() << std::endl;
					int i = 0;
				}
				ee.at(i) = std::clamp(ee.at(i), min, max);
			}
		}
	public:
		sptr<bitmap<res>> Develop(payloads* pays_allgen,exindicesWithHead* terminates) {

			

			sptr<bitmap<res>> ret(new bitmap<res>());
			int hei = 0;
			for (exindex tnowi : *terminates) {
				bool debugt = true;
				payload tnow = pays_allgen->at(tnowi);//���݂̃m�[�h�@�I�[����n�߂�
				hvec3 color = hvec3::Zero();

				while (1) {

					//�������s��
					MulHvec(color, tnow.GetContent().scale());
					AddHvec(color, tnow.GetContent().translate());

					//std::cout <<debugt<<"\t" << tnowi << "\t" << color.at(0).operator float() << "\t" << color.at(1).operator float() << "\t" << color.at(2).operator float() << std::endl;
					//debugt = false;
					//int xx = 61, yy = 50;
					//int resa = 128;
					//if (tnowi == xx + (resa - 1 - yy) * resa) {
					//	//std::cout << debugt << "\t" << tnowi << "\t" << color.at(0).operator float() << "\t" << color.at(1).operator float() << "\t" << color.at(2).operator float() << std::endl;
					//	int aa = 0;
					//}

					//�������I�[�Ȃ�
					if (tnow.parent == std::numeric_limits<exindex>::max()) {
						ClampHvec3(color);//�F�̈�֊ۂ߂�
						ret->at(tnowi) = color;

						break;
					}
					tnowi = tnow.parent;
					tnow = pays_allgen->at(tnowi);//�e�Ɉړ�����
				}
				
			}

			//for (exindex i = 0; i < res * res;i++) {
			//	ret->at(i) = pays_allgen->at(i).GetContent();
			//}

			return ret;
		}

	};
};