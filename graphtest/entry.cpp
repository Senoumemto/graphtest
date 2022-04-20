
#include"general.hpp"
#include"sub.hpp"

using namespace std;
using namespace Eigen;

#define CORE_NUM 1
#define ROOTER_CACHE (32*32)
#define ANYHIT_CACHE (ROOTER_CACHE)
#define MATERIALER_CACHE (ROOTER_CACHE)
#define CAMERA_RESOLUTION_SQRT 32

/*
tlasをアウターからなんとか構築し　それにレイトレース処理を行うことでrayHierarchyに変換　それを現像処理することでフレームを作成する
アウターからコマンドを使ってカメラとblasとそれぞれの変換を送信　tlasを構成する
*/

//装置たち
struct {
	toolkit::rooter<ROOTER_CACHE> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<ANYHIT_CACHE> anyhit;
	toolkit::materialer<MATERIALER_CACHE> materialer;
}machines;


//手順0(pre-phase)  アウターで行われるデータ構造の準備
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {
	prephaseRez rez;
	//手順0(事前処理) モデルを読み込んでblasを作成する&カメラを作成する
	dmod model;
	//ModLoader("../monkey.dae", model);
	MakeTestSquare(model);
	sptr<blas> obj(new blas(model));
	rez.objs.push_back(obj);


	rez.cam.reset(new camera(CAMERA_RESOLUTION_SQRT, CAMERA_RESOLUTION_SQRT, -1.0));

	return rez;
}


//手順1(upload-phase) blasやカメラをグラボに登録しrooterやbroad/narrow-phaserに登録(グラボ内部ではtlasと0-gen レイが作製される)
struct regphaseRez {
	sptr<tlas> scene;
};
void RegPhase(const vector<sptr<blas>>& objs, const sptr<camera>& cam) {

	//tlasを作製
	sptr<tlas> scene(new tlas);
	for (const auto& obj : objs)
		scene->push_back(make_pair(hmat4::Identity(), obj));//blasとその変換を登録

	machines.rooter.RegisterRays(*cam);

	//トレーサーにtlasをそれにインストール
	machines.broadphaser.ptlas = scene;

	machines.narrowphaser.ptlas = scene;
}

////手順2(broadphase) 登録されたデータを用いてブロードフェーズを行う
//sptr<broadphaseResults> BroadPhase() {
//	//手順3(broadphase)  broadphaseを行う　結果としてレイと広域衝突結果(gindex)のペアのリストを受け取る
//	sptr<broadphaseResults> bprez(new broadphaseResults);//広域衝突結果パッケージ
//	while (machines.rooter) {
//		ray r = machines.rooter.Get();//レイを一つ配ってもらう
//
//		auto r_sHits = machines.broadphaser.Raytrace({ r });//ブロードフェーズを行い広域衝突情報を受け取る
//		for (const auto& h : r_sHits)
//			bprez->push_back(broadphaseResultElement(r, h));//ヒット情報を結果パッケージに入れる
//	}
//
//	return bprez;
//}

int main() {

	auto preRez = PrePhase();//アウターがデータを用意する(blasとcam)


	RegPhase(preRez.objs, preRez.cam);//アウターがデータをグラボに登録(rooterへcam->0 gen raysが、broadphaserとnarrowphaserへblas-es->tlasが)


	//auto bpRez = BroadPhase();	//done命令があればグラボがブロードフェーズを行う
	auto generation = machines.rooter.GetGeneration();//rooterから世代を受け取る

	//auto npRez = machines.narrowphaser.RayTrace(*bpRez);//ブロードフェーズ結果からナローフェーズを行う
	//auto closestHits = machines.anyhit.Anyhit(*npRez);//レイの遮蔽を計算しclosest-hitを計算する
	//auto rayPayloads=machines.materialer.Shading(*closestHits);//レイの表面での振る舞いを計算 next gen raysを生成

	return 0;
}