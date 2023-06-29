#pragma once

#define _SILENCE_NONFLOATING_COMPLEX_DEPRECATION_WARNING

#include<fstream>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
//#include <half.hpp>
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


#define CEREAL_THREAD_SAFE 1
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/list.hpp>


using word = int16_t;
using uword = uint16_t;
using halff = double;
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
	vec3(const T& x, const T& y, const T& z) {
		this->at(0) = x;
		this->at(1) = y;
		this->at(2) = z;
	}

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


//std::pairをハッシュする
struct HashPair {

	static size_t m_hash_pair_random;

	template<class T1, class T2>
	size_t operator()(const std::pair<T1, T2>& p) const {

		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);

		size_t seed = 0;
		seed ^= hash1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hash2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};

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
//blasツリーを構成するノード 節か葉のどちらかを区別するフラグが入っている
struct blasnode {
	aabb data;
	bool isVertex;//節ノードである
};
using nodetree = std::vector<blasnode>;

class blas{
public:
	nodetree tree;//要素は1番から始まりn番ノードの左子ノードは2n 右子ノードは2n+1である
	hmod triangles;
	
	aabb MakeAABB(const hmod::iterator& b, const hmod::iterator& e);
	void innerSort(hmod::iterator b, hmod::iterator e,int axis);
public:
	blas(const dmod& d);
};
//光線
class ray :public std::pair<hvec3, hvec3> {
	using super = std::pair<hvec3, hvec3>;
protected:
	std::optional<exindex> id;//rooterに登録されるまではidなし
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
//親を持つレイ シェーダーからrooterまでのnextgenの表現に使ってね
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

//カメラ
class camera : public rays {
public:
	//1x1の解像度h*vのスクリーンを焦点からdistだけ離したカメラを作成する
	camera(size_t h, size_t v, double dist, double asp = 1.0);
	camera(size_t siz);
	camera();

	static halff CalcDistFromFov(const halff& fov);
};

//blasとその変換を登録する構造　sceneに対応する
class las :public std::vector<std::pair<hmat4, sptr<blas>>>{
	using blasset = std::pair<hmat4, sptr<blas>>;
	using super = std::vector<blasset>;
public:
	las(const super&s):super(s){}
	las():super(){}
};

//g-index tlasからtriをたどれるindex
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

//uvtとintoExtend,vsTriangleの返り値
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

//レイの属性として親とコンテンツを持つ
template<typename contentType>class payloadbase {

	contentType content;

public:
	virtual payloadbase& InstallContent(const contentType& c) {
		this->content = c;

		return *this;
	}
	const contentType& GetContent()const { return content; }

	exindex parent;//親を示すexindex(全世代)


};

class payloadContent :private std::pair<hvec3, hvec3> {
	using super = std::pair<hvec3, hvec3>;
	
public:
	hvec3& scale() { return this->first; }//色空間の拡縮(反射)
	hvec3& translate() { return this->second; }//色の並進(発光)

	const hvec3& scale() const{ return this->first; }//色空間の拡縮(反射)
	const hvec3& translate() const{ return this->second; }//色の並進(発光)

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

//bitmapを表す
using bitmapx = std::vector<hvec3>;


//グラボの機能を実装
namespace toolkit {
	//メモリコレクション
	template<size_t ATTRIB_SIZE>using attributeFramework = std::array<halff, ATTRIB_SIZE>;
	template<typename attribute>using triangleAttributeFramework = std::array<attribute, 3>;
	template<typename attribute>using triangleAttributesFramework = std::vector<triangleAttributeFramework<attribute>>;
	template<typename attribute>using attributesSetFramework = std::vector<triangleAttributesFramework<attribute>>;
	template<typename payload_type,size_t RAYNUM_LIMIT_ALLGEN,size_t RAYNUM_LIMIT_GENERATION,size_t RAYNUM_LIMIT_TERMINATES,size_t ATTRIB_SIZE,size_t BLASNUM_LIMIT,size_t TRIANGLES_NUM_LIMIT>class memoryCollection {
		using payloads = std::vector<payload_type>;
		using attribute = attributeFramework<ATTRIB_SIZE>;
		using triangleAttribute = triangleAttributeFramework<attribute>;
		using triangleAttributes = triangleAttributesFramework<attribute>;
		using attributesSet = attributesSetFramework<attribute>;//blasid-attributesのリスト

		uptr<rays> allows_allgen;//全世代のrayリスト(グローバル空間)
		uptr<payloads>payloads_allgen;//全世代のペイロードリスト

		uptr<closesthits>closesthits_gen;//今世代のclosesthit

		uptr<exindicesWithHead> terminates_allgen;//終端になるレイ(つまり光の発生点)の集合

		uptr<attributesSet> attributes;//面属性

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


	//レイを配布してくれるやつ
	template<size_t cachesize,typename paytype>class generator {
		using payloads = std::vector<paytype>;

		//sptr<payed_rays<paytype>> cache;
		//ここをメモリに接続する

		exindex MakeUniqueRayId(){
			static exindex id = 0;
			return id++;
		}
		exindex nowhead;//追加されるレイの頭位置
		exindex genhead;//次世代の頭位置
	public:
		generator():nowhead(0),genhead(0) {}

		//レイ集合を登録する　親のいない(0世代のレイを追加)=カメラ
		void RegisterRays(const rays& rs, rays* rays_allgen, payloads* pays_allgen) {
			for (exindex i = 0; i < rs.size(); i++) {
				rays_allgen->at(nowhead + i) = rs.at(i);
				rays_allgen->at(nowhead + i).indexed(nowhead + i);

				pays_allgen->at(nowhead + i).parent = std::numeric_limits<exindex>::max();
			}
			nowhead += rs.size();
		}
		//レイ集合を登録する　親つき=nextgen
		void RegisterRays(const parentedRays& rs, rays* rays_allgen,payloads* pays_allgen) {
			for (exindex i = 0; i < rs.size(); i++) {
				rays_allgen->at(nowhead + i) = rs.at(i).GetRay();
				rays_allgen->at(nowhead + i).indexed(nowhead + i);

				pays_allgen->at(nowhead + i).parent = rs.at(i).index();//ペイロードに親情報を追加
			}
			nowhead += rs.size();
		}

		rays GetGeneration(exindex& retGenhead, rays* rays_allgen,exindex& generationsize) {
			retGenhead = exindex(genhead);
			generationsize = nowhead - genhead;
			//世代頭から現在ヘッドまでを取り出す
			rays ret(nowhead - genhead);
			for (int i = genhead; i < nowhead; i++)
				ret.at(i - genhead) = rays_allgen->at(i);
			//= rays(cache->begin() + genhead, cache->begin() + nowhead, [](const std::pair<ray, paytype>& p) {return p.second; });
			//次世代用に吐き出し
			genhead = nowhead;

			return ret;
		}

		//レイのサイズを返す
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
				halff t0 = (box.min()[a] - r.org()[a]) * invD;//近面
				halff t1 = (box.max()[a] - r.org()[a]) * invD;//遠面
				if (invD < 0.0f)
					std::swap(t0, t1);
				tmin = t0 > tmin ? t0 : tmin;//近面のうちより遠いもの
				tmax = t1 < tmax ? t1 : tmax;//遠面のうちより近いもの
				//std::cout << tmin << "\t" << tmax << std::endl;

				//if (tmax < tmin)//交差しているならだめ
				if (tmin - tmax > param_aabbMargin)//交差しているならダメ　つまりtminの方が大きければだめ　ただしマージン有
					return false;
			}
			return true;
		}

		//レイ1つに対してレイトレを行う 結果はレイごとの広域衝突リスト(gindex)
		std::deque<gindex> Raytrace(const std::array<ray, coren>& rs) {
			std::deque<gindex> ret;
			//コアごとにレイトレを行う 結果はレイごとの広域衝突リスト(gindex)
			for (int i = 0; i < coren; i++) {
				auto each = cores.at(i).Raytrace(rs.at(i));
				for (auto& e : each)
					ret.push_back(e);
			}

			return ret;
		}
	public:
		//sptr<tlas>ptlas;//ここにtlasをインストールして使う


		//トレーサーコア
		struct core {
			sindex nowi;
			bool isInv;//ただいま逆行中 移行できるようになるまで逆行を続ける
			broadphaser<coren>* parent;

			//rで親のplasに対してレイトレーシングを行い広域衝突情報のリストを返す
			std::deque<std::pair<ray,gindex>> Raytrace(const ray& r_grobal,const las* plas) {
				using evec3 = Eigen::Vector3<halff>;
				using evec4 = Eigen::Vector4<halff>;
				//using namespace half_float::literal;

				std::deque<std::pair<ray,gindex>> ret;
				//blasごとに行う
				for (sindex blasid = 0; blasid < plas->size();blasid++) {
					const auto& blasset = plas->at(blasid);
					//変換をレイに適用する
					ray r;
					r.indexed(r_grobal.index());//同一のレイである
					r.way() = hvec3::Make(evec4(blasset.first * evec4(r_grobal.way().x(), r_grobal.way().y(), r_grobal.way().z(), 0.0)).normalized(), false);
					r.org() = hvec3::Make(evec4(blasset.first * evec4(r_grobal.org().x(), r_grobal.org().y(), r_grobal.org().z(), 1.0)), true);


					const sindex leafhead = (blasset.second->tree.size() + 1) / 2;//葉ノードの一番最初

					nowi = 1;//ルートに戻る
					isInv = false;
					do {
						//std::cout << "今のノード\t" << nowi << std::endl;
						//そもそも逆行中なら
						if (isInv) {
							//逆行解除の条件は左ノードであること
							if (nowi % 2 == 0) {
								isInv = false;
								nowi = nowi + 1;//さらに移行
							}
							else nowi = (nowi - 1) / 2;//そうでなければ逆行
						}
						else {
							if (parent->vsAABB(r, blasset.second->tree.at(nowi-1).data)) {
								//ヒットなら
								if (blasset.second->tree.at(nowi-1).isVertex)
									nowi = nowi * 2;//かつ節なら順行　2n
								else {
									ret.push_back(std::make_pair(r,std::make_pair(blasid,(sindex)(nowi-leafhead))));//かつは葉なら予約
									if (nowi % 2) {
										nowi = (nowi - 1) / 2;//右なら逆行
										isInv = true;
									}
									else nowi = nowi + 1;//左なら移行
								}
							}
							else {
								//ヒットでなければ
								if (nowi % 2 == 0) {
									//かつ左ノードであれば移行
									nowi = nowi + 1;
								}
								else {
									//かつ右ノードであれば逆行
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

			//rsから複製してrを得る
			for (const ray& r : rs) {
				auto r_sHits = cores.front().Raytrace(r,plas);//ブロードフェーズを行い広域衝突情報を受け取る

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
		//sptr<tlas>ptlas;//ここにtlasをインストールして使う
		halff param_extendSize;

		sptr<narrowphaseResults> RayTrace(const broadphaseResults& bprez,const halff param_ignoreNearHit, const halff param_ignoreParallelHit, const las* plas);
		narrowphaser(const halff& extendSize):param_extendSize(extendSize){}
	};


	template<exindex cachesize>class obstructer {
	protected:
	public:
		exindex Anyhit(const narrowphaseResults& nprez,exindex genhead, std::unordered_map<exindex, closesthit>& closests_gen) {
			for (const auto& rez : nprez) {
				//closestがextendのとき、挑戦者がnot extendかtがより大きければ更新
				//closestがnot extendのとき、挑戦者がnot extendでかつtがより大きければ
				auto itex = closests_gen.find(rez.r.index());//検索する
				if (itex == closests_gen.cend()) {
					closests_gen[rez.r.index()] = rez;
				}
				else {
					bool tLarger = rez.vsTriRez.uvt().at(2) < itex->second.vsTriRez.uvt().at(2);//挑戦者のtが大きい

					if (itex->second.vsTriRez.intoExtend() && (!rez.vsTriRez.intoExtend() || tLarger) || !itex->second.vsTriRez.intoExtend() && (!rez.vsTriRez.intoExtend() && tLarger)) {//上の条件で再登録
						itex->second = rez;
					}
				}
			}

			//for (const auto& c : *cache) {
			//	if (c.uvt.at(2) != std::numeric_limits<halff>::infinity())
			//		std::cout << "index " << c.r.index() << std::endl;
			//}

			return 1;
		}
		void InstallGeneration(const rays& nowgen,exindex genhead, closesthits* closests_gen) {
			//using namespace half_float::literal;
			for (const ray& r : nowgen) {
				closests_gen->at(r.index() - genhead).r = r;//レイを挿入
				closests_gen->at(r.index() - genhead).vsTriRez = std::make_pair(hvec3({ 0.0, 0.0, std::numeric_limits<halff>::infinity() }),true);//無限遠で交差 何もない状態ではextendを受け入れる
			}
		}

		obstructer(){}
	};

	template<exindex cachesize,sindex brunchsize,typename ATTRIB_TYPE>class materializer {


		void ApplyToRay(const hmat4& m, ray& r) {
			//using namespace half_float::literal;
			using evec4 = Eigen::Vector4<halff>;
			//r.indexed(r.index());インデックスは未だ割り振られてない
			r.way() = hvec3::Make(evec4(m * evec4(r.way().x(), r.way().y(), r.way().z(), 0.0)).normalized(), false);
			r.org() = hvec3::Make(evec4(m * evec4(r.org().x(), r.org().y(), r.org().z(), 1.0)), true);
		}
	public:
		//一つのレイから作られた次世代 ヘッド付き
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
		//シェーダーリスト blas-idを用いる
		struct shaderlist :public std::vector<shader> {
			shader miss;
		};

		shaderlist mats;//shader HitShader, MissShader;

		const payloads* Shading(const closesthits& hits,parentedRays& nextgen,bool arrowNextgen, payloads* pays_allgen,exindicesWithHead* terminates,size_t anyhitsize,const las* plas,const attributesSetFramework<ATTRIB_TYPE>* attribSet) {
			for (size_t i = 0; i < anyhitsize; i++) {
				const auto r = hits.at(i);
				brunch nextbrunch;//今回生じるレイ
				bool isTerminate = false;//このレイは終端であるか?
				//レンダリング　対応したシェーダを呼び出してペイロードリストに追加
				pays_allgen->at(r.r.index()).InstallContent((r.vsTriRez.uvt().at(2) != std::numeric_limits<halff>::infinity()) ? mats.at(r.tri.blasId()).operator()(r, nextbrunch, plas,isTerminate) : mats.miss(r, nextbrunch, plas,isTerminate));
				
				//終端か次世代が許可されなければ
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

};

template<size_t DIM>using uvec = Eigen::Vector<double, DIM>;
using index = size_t;
template<size_t SIZ>class arrow :public std::pair<uvec<SIZ>, uvec<SIZ>> {
private:
	using super = std::pair<uvec<SIZ>, uvec<SIZ>>;

public:
	//始点
	decltype(arrow::first)& org() { return this->first; }
	const decltype(arrow::first)& org()const { return this->first; }
	//終点
	decltype(arrow::second)& dir() { return this->second; }
	const decltype(arrow::second)& dir() const { return this->second; }

	//org dirの順で初期化
	arrow() = default;
	arrow(const decltype(arrow::first)& orign, const decltype(arrow::second)& direction) :super(orign, direction) {}
	arrow(const arrow& val) = default;

	arrow& operator=(const arrow& val) {
		this->org() = val.org();
		this->dir() = val.dir();

		return *this;
	}


	template<size_t CASTSIZ> operator arrow<CASTSIZ>() const {

		arrow<CASTSIZ> ret;
		ret.org() = (decltype(ret.org()))(this->org());
		ret.dir() = (decltype(ret.org()))(this->dir());

		if constexpr (SIZ < CASTSIZ) {
			//zero埋めする
			for (index i = SIZ; i < CASTSIZ; i++) {
				ret.org()(i) = 0.;
				ret.dir()(i) = 0.;
			}
		}

		return ret;
	}

	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) const {
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->org()[d]);
		}
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->dir()[d]);
		}

		//archive(this->org().x(), this->org().y(), this->org().z(),
		//this->dir().x(), this->dir().y(), this->dir().z());
	}
	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) {
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->org()[d]);
		}
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->dir()[d]);
		}
		//archive(this->org().x(), this->org().y(), this->org().z(),
		//this->dir().x(), this->dir().y(), this->dir().z());
	}
};
using arrow2 = arrow<2>;
using arrow3 = arrow<3>;
using arrow4 = arrow<4>;

//projectorRefractionDicのヘッダ構造
class projRefraDicHeader {
public:
	size_t horizontalRes;//水平分解能　ピクセル数
	size_t verticalRes;//垂直分解能　ピクセル数
	size_t rotationRes;//回転分解能t つまり一周に何回投影するか　つまり分散数


	projRefraDicHeader(const size_t& hRes, const size_t& vRes, const size_t& rotRes) :
		rotationRes(rotRes), verticalRes(vRes), horizontalRes(hRes) {}
	projRefraDicHeader() {}

	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) const {
		archive(this->horizontalRes, this->verticalRes, this->rotationRes);
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(this->horizontalRes, this->verticalRes, this->rotationRes);
	}

	//ヘッダファイルを保存
	void SaveHeader(const std::string& path) const {
		std::ofstream ofs(path + ".head");
		cereal::BinaryOutputArchive o_archive(ofs);

		o_archive((projRefraDicHeader&)(*this));

		return;
	}
};
