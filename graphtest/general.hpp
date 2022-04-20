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

#include<Eigen/Core>
#include<Eigen/Dense>


using word = int16_t;
using uword = uint16_t;
using half = half_float::half;
using hmat4 = Eigen::Matrix4<half>;
using index = uint16_t;

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
};
using hvec3 = vec3<half>;

template<typename T>using tri = vec3<vec3<T>>;
template<typename T>using mod = std::vector<tri<T>>;

using htri = tri<half>;

using dmod = mod<double>;
using hmod = mod<half>;

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
	std::optional<index> id;//rooter�ɓo�^�����܂ł�id�Ȃ�
public:
	hvec3& org();
	hvec3& way();

	const hvec3& org()const;
	const hvec3& way()const;

	ray(const super& s) :super(s) {}
	ray() {}

	ray& indexed(index _id) { id = _id; return *this; }
	index index()const { return id.value(); }
};
using rays = std::vector<ray>;

//�J����
class camera : public rays {
public:
	//1x1�̉𑜓xh*v�̃X�N���[�����œ_����dist�����������J�������쐬����
	camera(size_t h, size_t v, double dist, double asp = 1.0);
};

//blas�Ƃ��̕ϊ���o�^����\���@scene�ɑΉ�����
class tlas :public std::vector<std::pair<hmat4, sptr<blas>>>{
	using blasset = std::pair<hmat4, sptr<blas>>;
	using super = std::vector<blasset>;
public:
	tlas(const super&s):super(s){}
	tlas():super(){}
};

//g-index tlas����tri�����ǂ��index
class gindex :public std::pair<index, index> {
	using super = std::pair<index, index>;
public:
	gindex():super(){}
	gindex(const super&s):super(s){}

	index& blasId() { return this->first; }
	index& triId() { return this->second; }
	const index& blasId() const{ return this->first; }
	const index& triId() const{ return this->second; }
};
using broadphaseResultElement = std::pair<ray, gindex>;
using broadphaseResults = std::deque<broadphaseResultElement>;

struct narrowphaseResultElement {
	ray r;
	gindex tri;
	hvec3 uvt;

	narrowphaseResultElement(){}
	narrowphaseResultElement(const broadphaseResultElement& bprez, const hvec3& uvtrez) :r(bprez.first), tri(bprez.second), uvt(uvtrez){}
};
using narrowphaseResults = std::deque<narrowphaseResultElement>;

class closesthit :public narrowphaseResultElement {
	using super = narrowphaseResultElement;
protected:
	bool enable;
public:
	closesthit():enable(false){}
	closesthit(const super& s) :super(s) { closesthit(); }

	void operator=(const super& s) {
		super::operator=(s);
		enable = true;
	}

	bool isEnable()const {
		return enable;
	}
};

using closesthits = std::vector<closesthit>;

using payload = bool;
using payloads = std::vector<payload>;

//�O���{�̋@�\������
namespace toolkit {
	//���C��z�z���Ă������
	class rooter {
		std::deque<ray> stack;
		index MakeUniqueRayId();
	public:
		rooter();

		//���C�W����o�^����@�J�������ǉ��ł���
		void RegisterRays(const rays& rs);

		operator bool();

		ray Get();
	};

	template<size_t coren>class broadphaser {
	protected:
		bool vsAABB(const ray& r, const aabb& box, half tmin = half(0.0), half tmax = std::numeric_limits<half>::infinity()) {

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
				half invD = half(1.0) / r.way()[a];
				half t0 = (box.min()[a] - r.org()[a]) * invD;
				half t1 = (box.max()[a] - r.org()[a]) * invD;
				if (invD < 0.0f)
					std::swap(t0, t1);
				tmin = t0 > tmin ? t0 : tmin;
				tmax = t1 < tmax ? t1 : tmax;
				//std::cout << tmin << "\t" << tmax << std::endl;

				if (tmax < tmin)
					return false;
			}
			return true;
		}
	public:
		sptr<tlas>ptlas;//������tlas���C���X�g�[�����Ďg��


		//�g���[�T�[�R�A
		struct core {
			index nowi;
			bool isInv;//�������܋t�s�� �ڍs�ł���悤�ɂȂ�܂ŋt�s�𑱂���
			broadphaser<coren>* parent;

			//r�Őe��ptlas�ɑ΂��ă��C�g���[�V���O���s���L��Փˏ��̃��X�g��Ԃ�
			std::deque<gindex> Raytrace(ray r) {
				std::deque<gindex> ret;
				//blas���Ƃɍs��
				for (index blasid = 0; blasid < parent->ptlas->size();blasid++) {
					const auto& blasset = parent->ptlas->at(blasid);
					//�ϊ������C�ɓK�p����
					//���܂͂Ƃ�܂Ȃ���

					const index leafhead = (blasset.second->tree.size() + 1) / 2;//�t�m�[�h�̈�ԍŏ�

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
									ret.push_back(std::make_pair(blasid,(index)(nowi-leafhead)));//���͗t�Ȃ�\��
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

		broadphaser():cores() {
			for (core& i : cores)
				i.parent = this;
		}

		//���C1�ɑ΂��ă��C�g�����s�� ���ʂ̓��C���Ƃ̍L��Փ˃��X�g(gindex)
		std::deque<gindex> Raytrace(const std::array<ray,coren>& rs) {
			std::deque<gindex> ret;
				//�R�A���ƂɃ��C�g�����s�� ���ʂ̓��C���Ƃ̍L��Փ˃��X�g(gindex)
			for (int i = 0; i < coren; i++) {
				auto each = cores.at(i).Raytrace(rs.at(i));
				for (auto& e : each)
					ret.push_back(e);
			}

			return ret;
		}
	};


	class narrowphaser {
	protected:
		std::optional<hvec3> vsTriangle(const ray& r, const htri& tri);
	public:
		sptr<tlas>ptlas;//������tlas���C���X�g�[�����Ďg��

		sptr<narrowphaseResults> RayTrace(const broadphaseResults& bprez);
	};


	template<index cachesize>class anyhit {
	protected:
		sptr<closesthits> cache;
	public:
		sptr<const closesthits> Anyhit(const narrowphaseResults& nprez) {
			for (const auto& rez : nprez) {
				if (cache->at(rez.r.index()).isEnable()) {
					if (rez.uvt.at(2) < cache->at(rez.r.index()).uvt.at(2))//�o�^�ς݂�t���rez��t����������΍ēo�^
						cache->at(rez.r.index()) = rez;
					
				}
				else cache->at(rez.r.index()) = rez;//�o�^����Ă��Ȃ���Ζⓚ���p�œo�^

			}

			for (const auto& c : *cache) {
				if (c.isEnable())
					std::cout << "index " << c.r.index() << std::endl;
			}

			return cache;
		}

		anyhit():cache(new closesthits(cachesize)) {}
	};


	template<index cachesize>class materialer {
		
		sptr<payloads> cache;
		payload Shader(const closesthit& att) {
			return (payload)att.isEnable();
		}

	public:
		sptr<const payloads> Shading(const closesthits& hits) {
			for (const auto& r : hits) {
				if (!r.isEnable())continue;

				cache->at(r.r.index()) = Shader(r);
			}

			return cache;
		}

		materialer():cache(new payloads(cachesize)){}
	};
};