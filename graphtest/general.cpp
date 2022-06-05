#include "general.hpp"

using namespace std;
using evec3 = Eigen::Vector3<halff>;

htri ToHaabb(const tri<double>& in) {
	htri ret;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			ret.at(i).at(j) = in.at(i).at(j);

	return ret;
}
//入力を一番近い入力より大きな2のn乗にする
size_t To2noNjou(size_t i) {
	size_t ret = 1;
	while (ret < i)ret *= 2;

	return ret;
}

htri unable_tri({ hvec3({ halff(1.0),halff(1.0),halff(1.0) }),hvec3({halff(1.0),halff(1.0),halff(1.0)}) ,hvec3({halff(1.0),halff(1.0),halff(1.0)})});//かさましの無効な三角形

hvec3 GetCenter(const htri& h) {
	hvec3 ret({ halff(0),halff(0),halff(0) });
	for (const auto& i : h)
		for (int j = 0; j < 3; j++)
			ret.at(j) += i.at(j);

	ret.x() /= 3;
	ret.y() /= 3;
	ret.z() /= 3;

	return ret;

}

blas::blas(const dmod& d) {
	triangles.resize(To2noNjou(d.size()));//2のn乗の要素数が必要だ！
	//まずツリーにモッドを張り付ける
	for (sindex i = 0; i < triangles.size(); i++)
		if (i < d.size())
			triangles.at(i)=ToHaabb(d.at(i));
		else triangles.at(i)=unable_tri;

	//ノードのサイズを確定する
	this->tree.reserve(triangles.size() * 2 - 1);

	//triが一つなら終わり
	if (triangles.size() <= 1)return;

	int axis = 0;//ソートされる場合の軸

	//階数=0~からループ
	for (int level=0;;level++,axis=(axis + 1) % 3) {
		//予約された範囲でaabbを作る

		const sindex levlength = triangles.size() / pow(2, level);//範囲の長さを算出

		//範囲は2^level個だけ存在する
		for (int l = 0; l < pow(2, level); l++) {
			//n個めの範囲はlength*n~length*(n+1)
			//それでboxを作る
			blasnode thisnode = { MakeAABB(this->triangles.begin()+(levlength*l), this->triangles.begin() + (levlength * (l+1))),levlength != 1};
			this->tree.push_back(thisnode);

			//AABBを指定された軸で二分したい　センターでソートして2分
			std::sort(this->triangles.begin() + (levlength * l), this->triangles.begin() + (levlength * (l + 1)), [&](const tri<halff>& a, const tri<halff>& b) {
				hvec3 ac = GetCenter(a), bc = GetCenter(b);

				return ac.at(axis) > bc.at(axis);
				});

		}

		if (levlength == 1)break;
	}

	//cout << "triangles" << endl;
	//for (auto& tr : this->triangles) {
	//	for (auto& p : tr) {
	//		for (auto& e : p)
	//			cout << e << "\t";
	//		cout << endl;
	//	}
	//	cout << endl;
	//}
}

//指定された軸でソートして2分　次の軸でソートする
void blas::innerSort(hmod::iterator b, hmod::iterator e, int axis) {
	sindex dist = std::distance(b, e);
	//この範囲でaabbを作る ノード数が1でなければ節
	blasnode thisnode = { MakeAABB(b, e),dist != 1 };
	this->tree.push_back(thisnode);

	//AABBを指定された軸で二分したい　センターでソートして2分
	std::sort(triangles.begin(), triangles.end(), [&](const tri<halff>& a, const tri<halff>& b) {
		hvec3 ac = GetCenter(a), bc = GetCenter(b);

		return ac.at(axis) > bc.at(axis);
		});

	//distが1なら終了
	if (dist <= 1) {
		return;
	}

	innerSort(b, b + (dist / 2), (axis + 1) % 3);
	innerSort(b + (dist / 2), e, (axis + 1) % 3);
}

hvec3& ray::org() {
	return this->first;
}
hvec3& ray::way() {
	return this->second;
}

const hvec3& ray::org()const {
	return this->first;
}
const hvec3& ray::way()const {
	return this->second;
}

camera::camera(size_t h, size_t v, double dist,double asp) {
	this->reserve(h * v);

	for (int y = 0; y < v; y++)
		for (int x = 0; x < h; x++) {
			//スクリーンの位置は((2/res)*i+(1/res))-1
			double scx = ((2.0 / h) * x + (1.0 / h) - 1.0) * asp;
			double scy = (2.0 / v) * y + (1.0 / v) - 1.0;
			double scz = dist;

			//orgが0 wayがスクリーンの正規化
			Eigen::Vector3d scnormed = Eigen::Vector3d(scx, scy, scz).normalized();

			this->push_back(ray({ hvec3({halff(0),halff(0),halff(0)}),hvec3({halff(scnormed.x()),halff(scnormed.y()),halff(scnormed.z())}) }));
		}
}

aabb blas::MakeAABB(const hmod::iterator& b, const hmod::iterator& e) {
	aabb ret(std::make_pair(b->front(), b->front()));//暫定座標を決める

	//含まれるポリゴンごとに
	for (hmod::iterator i = b; i != e; i++) {
		//ポリゴンの中で最大座標を決める
		hvec3 mins = i->front(), maxs = i->front();//暫定の最大/最小
		{
			for (int axis = 0; axis < 3; axis++) {
				for (int p = 0; p < 3; p++) {
					mins.at(axis) = std::min(mins.at(axis), i->at(p).at(axis));//暫定よりもさらに小さいものがあればそれを挿入
					maxs.at(axis) = std::max(maxs.at(axis), i->at(p).at(axis));
				}
			}
		}

		//軸ごとに暫定の最小/最大を比較する
		for (int axis = 0; axis < 3; axis++) {
			ret.min().at(axis) = std::min(ret.min().at(axis), mins.at(axis));
			ret.max().at(axis) = std::max(ret.max().at(axis), maxs.at(axis));
		}
	}

	return ret;
}

optional<hvec3> toolkit::narrowphaser::vsTriangle(const ray& ray, const htri& tri) {
	//std::cout << "ray" << std::endl;
	//for (const auto& i : ray.way())
	//	std::cout << i << "\t";
	//std::cout << std::endl;
	//for (const auto& i : ray.org())
	//	std::cout << i << "\t";
	//std::cout << "\n" << std::endl;

	//cout << "ttri" << endl;
	//for (const auto& i : tri) {
	//	for (const auto& e : i)
	//		cout << e << "\t";
	//	cout << endl;
	//}
	//cout << endl;
	using namespace half_float::literal;
	// 微小な定数
	constexpr halff kEpsilon = std::numeric_limits<halff>::epsilon();
	const halff margined1 = 1.0_h + kEpsilon;//1.0より大きい最小の数
	const halff doublemargined1 = margined1 +  kEpsilon;//1.0より大きい最小の数より大きい最小の数
	constexpr halff minusEps = -std::numeric_limits<halff>::epsilon();

	using evec3 = Eigen::Vector3<halff>;
	using namespace half_float::literal;
	//演算系に
	evec3 eway(ray.way().data()), eorg(ray.org().data());

	evec3 e1 = evec3(tri.at(1).data()) - evec3(tri.at(0).data()), e2 = evec3(tri.at(2).data()) - evec3(tri.at(0).data());

	evec3 alpha = eway.cross(e2);
	halff det = e1.dot(alpha);

	if (std::abs(det) < kEpsilon)return nullopt;

	halff invdet = halff(1) / det;
	evec3 r = eorg - evec3(tri.at(0).data());

	halff u = alpha.dot(r) * invdet;
	if (u < minusEps || u>margined1)return nullopt;

	evec3 beta = r.cross(e1);

	halff v = eway.dot(beta) * invdet;
	if (v < minusEps || u + v>doublemargined1)return nullopt;

	halff t = e2.dot(beta) * invdet;
	if (t < minusEps)return nullopt;

	return hvec3({ u, v, t });
}

//表面からの入射角のcos
halff incidenceAngleCos(const ray& ray, const htri& tri) {
	auto norm = ((evec3(tri.at(1).data()) - evec3(tri.at(0).data())).cross(evec3(tri.at(2).data()) - evec3(tri.at(0).data()))).normalized();
	auto direction = evec3(ray.way().data()).normalized();

	return norm.dot(direction);
}

sptr<narrowphaseResults> toolkit::narrowphaser::RayTrace(const broadphaseResults& bprez, const halff param_ignoreNearHit,const halff param_ignoreParallelHit) {
	sptr<narrowphaseResults> rez(new narrowphaseResults);
	using namespace half_float::literal;

	for (const auto& bp : bprez) {
		auto uvt = vsTriangle(bp.first, ptlas->at(bp.second.blasId()).second->triangles.at(bp.second.triId()));
		auto inciCos = incidenceAngleCos(bp.first, ptlas->at(bp.second.blasId()).second->triangles.at(bp.second.triId()));

		//ヒットしたら
		if (uvt.has_value()) {
			if (uvt.value().at(2) > param_ignoreNearHit) {//無視値より大きい
				if (std::abs(inciCos) > param_ignoreParallelHit)
					rez->push_back(narrowphaseResultElement(bp, uvt.value()));
				else
					cout << "ignored with parallel" << endl;
			}
		}
	}

	return rez;
}