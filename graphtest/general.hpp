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
};

//blasとその変換を登録する構造　sceneに対応する
class tlas :public std::vector<std::pair<hmat4, sptr<blas>>>{
	using blasset = std::pair<hmat4, sptr<blas>>;
	using super = std::vector<blasset>;
public:
	tlas(const super&s):super(s){}
	tlas():super(){}
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

struct narrowphaseResultElement {
	ray r;
	gindex tri;
	hvec3 uvt;

	narrowphaseResultElement(){}
	narrowphaseResultElement(const broadphaseResultElement& bprez, const hvec3& uvtrez) :r(bprez.first), tri(bprez.second), uvt(uvtrez){}
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

	payloadContent(const hvec3& scale, const hvec3& trans) { 
		this->scale() = scale;
		this->translate() = trans;
	}
	payloadContent(const std::array<halff,3>& scale, const std::array<halff, 3>& trans):payloadContent(hvec3(scale), hvec3(trans)) {}
	payloadContent(const hvec3& scale):payloadContent(scale, hvec3::Zero()) {}
	payloadContent(const std::array<halff,3>& scale):payloadContent(hvec3(scale)) {}
	payloadContent(const super&s):super(s){}
	payloadContent(const payloadContent&c):payloadContent(c.scale(),c.translate()){}
	payloadContent(){}
};
using payload = payloadbase<payloadContent>;
using payloads = std::vector<payload>;

template<typename paytype>using payed_ray = std::pair<ray, paytype>;
template<typename paytype>using payed_rays = std::vector<payed_ray<paytype>>;

//bitmapを表す
template<size_t res>struct bitmap :public std::array<hvec3, res* res>{

};

//グラボの機能を実装
namespace toolkit {
	//メモリコレクション
	template<typename payload_type,size_t RAYNUM_LIMIT_ALLGEN,size_t RAYNUM_LIMIT_GENERATION,size_t RAYNUM_LIMIT_TERMINATES>class memoryCollection {
		using payloads = std::vector<payload_type>;

		uptr<rays> rays_allgen;//全世代のrayリスト(グローバル空間)
		uptr<payloads>pays_allgen;//全世代のペイロードリスト

		uptr<closesthits>closests_gen;//今世代のclosesthit

		uptr<exindicesWithHead> terminates;//終端になるレイ(つまり光の発生点)の集合
	public:
		memoryCollection() {
			rays_allgen.reset(new rays(RAYNUM_LIMIT_ALLGEN));
			pays_allgen.reset(new payloads(RAYNUM_LIMIT_ALLGEN));
			closests_gen.reset(new closesthits(RAYNUM_LIMIT_GENERATION));
			terminates.reset(new exindicesWithHead(RAYNUM_LIMIT_TERMINATES));
		}

		rays* GetAllGenRays() { return rays_allgen.get(); }
		payloads* GetAllGenPayloads() { return pays_allgen.get(); }
		closesthits* GetNowGenClosests() { return closests_gen.get(); }
		exindicesWithHead* GetTerminates() { return terminates.get(); }
	};


	//レイを配布してくれるやつ
	template<size_t cachesize,typename paytype>class rooter {
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
		rooter():nowhead(0),genhead(0) {}

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
				halff t0 = (box.min()[a] - r.org()[a]) * invD;
				halff t1 = (box.max()[a] - r.org()[a]) * invD;
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
		sptr<tlas>ptlas;//ここにtlasをインストールして使う


		//トレーサーコア
		struct core {
			sindex nowi;
			bool isInv;//ただいま逆行中 移行できるようになるまで逆行を続ける
			broadphaser<coren>* parent;

			//rで親のptlasに対してレイトレーシングを行い広域衝突情報のリストを返す
			std::deque<std::pair<ray,gindex>> Raytrace(const ray& r_grobal) {
				using evec3 = Eigen::Vector3<halff>;
				using evec4 = Eigen::Vector4<halff>;
				using namespace half_float::literal;

				std::deque<std::pair<ray,gindex>> ret;
				//blasごとに行う
				for (sindex blasid = 0; blasid < parent->ptlas->size();blasid++) {
					const auto& blasset = parent->ptlas->at(blasid);
					//変換をレイに適用する
					ray r;
					r.indexed(r_grobal.index());//同一のレイである
					r.way() = hvec3::Make(evec4(blasset.first * evec4(r_grobal.way().x(), r_grobal.way().y(), r_grobal.way().z(), 0.0_h)), false);
					r.org() = hvec3::Make(evec4(blasset.first * evec4(r_grobal.org().x(), r_grobal.org().y(), r_grobal.org().z(), 1.0_h)), true);

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

		broadphaser():cores() {
			for (core& i : cores)
				i.parent = this;
		}

		sptr<broadphaseResults> Broadphase(const rays& rs) {

			sptr<broadphaseResults> ret(new broadphaseResults);

			//rsから複製してrを得る
			for (const ray& r : rs) {
				auto r_sHits = cores.front().Raytrace(r);//ブロードフェーズを行い広域衝突情報を受け取る

				for (const auto& g : r_sHits)
					ret->push_back(g);
			}

			return ret;
		}
	};


	class narrowphaser {
	protected:
		std::optional<hvec3> vsTriangle(const ray& r, const htri& tri);
	public:
		sptr<tlas>ptlas;//ここにtlasをインストールして使う

		sptr<narrowphaseResults> RayTrace(const broadphaseResults& bprez,const halff param_ignoreNearHit);
	};


	template<exindex cachesize>class anyhit {
	protected:
		closesthits* closests_gen;
	public:
		exindex Anyhit(const narrowphaseResults& nprez,exindex genhead, closesthits* closests_gen) {
			exindex anyhitsize = 0;
			for (const auto& rez : nprez) {
				if (rez.uvt.at(2) < closests_gen->at(rez.r.index() - genhead).uvt.at(2)) {//登録済みのtよりrezのtが小さければ再登録
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
				closests_gen->at(r.index() - genhead).r = r;//レイを挿入
				closests_gen->at(r.index() - genhead).uvt = hvec3({ 0.0_h, 0.0_h, std::numeric_limits<halff>::infinity() });//無限遠で交差
			}
		}

		anyhit(){}
	};

	template<exindex cachesize>class materialer {
	public:
		sptr<tlas>ptlas;
		using shader = std::function<payloadContent(const closesthit&, parentedRays&, exindicesWithHead*, sptr<tlas>)>;

		shader HitShader, MissShader;

		const payloads* Shading(const closesthits& hits,parentedRays& nextgen, payloads* pays_allgen,exindicesWithHead* terminates,size_t anyhitsize) {
			for (size_t i = 0; i < anyhitsize; i++) {
				const auto r = hits.at(i);
				pays_allgen->at(r.r.index()).InstallContent((r.uvt.at(2) != std::numeric_limits<halff>::infinity()) ? HitShader(r, nextgen, terminates, ptlas) : MissShader(r, nextgen, terminates, ptlas));
			}
			
			return pays_allgen;
		}

		materialer(){}
	};

	template<typename paytype,size_t res> class developper {
		//eeにerを掛け加える
		void MulHvec(hvec3& ee,const hvec3& er) {
			for (int i = 0; i < 3; i++)
				ee.at(i) *= er.at(i);
		}
		void AddHvec(hvec3& ee, const hvec3& er) {
			for (int i = 0; i < 3; i++)
				ee.at(i) += er.at(i);
		}
		void ClampHvec3(hvec3& ee,const halff& min=halff(0.0), const halff& max=halff(1.0)) {
			for (int i = 0; i < 3; i++)
				ee.at(i) = std::clamp(ee.at(i), min, max);
		}
	public:
		sptr<bitmap<res>> Develop(payloads* pays_allgen,exindicesWithHead* terminates) {

			sptr<bitmap<res>> ret(new bitmap<res>());
			int hei = 0;
			for (exindex tnowi : *terminates) {
				payload tnow = pays_allgen->at(tnowi);//現在のノード　終端から始める
				hvec3 color = hvec3::Zero();

				while (1) {
					//処理を行う
					MulHvec(color, tnow.GetContent().scale());
					AddHvec(color, tnow.GetContent().translate());

					//ここが終端なら
					if (tnow.parent == std::numeric_limits<exindex>::max()) {
						ClampHvec3(color);//色領域へ丸める
						ret->at(tnowi) = color;
						
						break;
					}
					tnowi = tnow.parent;
					tnow = pays_allgen->at(tnowi);//親に移動する
				}
				
			}

			//for (exindex i = 0; i < res * res;i++) {
			//	ret->at(i) = pays_allgen->at(i).GetContent();
			//}

			return ret;
		}

	};
};