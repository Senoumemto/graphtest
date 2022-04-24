
#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;

#define CORE_NUM 1
#define ROOTER_CACHE (128*128)
#define ANYHIT_CACHE (ROOTER_CACHE)
#define MATERIALER_CACHE (ROOTER_CACHE)
#define CAMERA_RESOLUTION 128

/*
tlasをアウターからなんとか構築し　それにレイトレース処理を行うことでrayHierarchyに変換　それを現像処理することでフレームを作成する
アウターからコマンドを使ってカメラとblasとそれぞれの変換を送信　tlasを構成する
*/

//装置たち
struct {
	toolkit::rooter<ROOTER_CACHE,payload> rooter;
	toolkit::broadphaser<CORE_NUM> broadphaser;
	toolkit::narrowphaser narrowphaser;
	toolkit::anyhit<ANYHIT_CACHE> anyhit;
	toolkit::materialer<MATERIALER_CACHE> materialer;
	toolkit::developper<payload, CAMERA_RESOLUTION> developper;
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
	ModLoader("../monkey.dae", model);
	//MakeTestSquare(model);
	sptr<blas> obj(new blas(model));
	rez.objs.push_back(obj);


	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -1.0));

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


int main() {

	auto preRez = PrePhase();//アウターがデータを用意する(blasとcam)


	RegPhase(preRez.objs, preRez.cam);//アウターがデータをグラボに登録(rooterへcam->0 gen raysが、broadphaserとnarrowphaserへblas-es->tlasが)

	index genhead;
	auto generation = machines.rooter.GetGeneration(genhead);//rooterから世代を受け取る

	machines.anyhit.InstallGeneration(generation, genhead);//anyhitに今世代の情報を通知してあげる
	cout << "Broadphase began" << endl;
	auto bpRez = machines.broadphaser.Broadphase(generation);//ブロードフェーズを行う　偽陽性を持つray,g-index結果を得る

	cout << "narrowphase began" << endl;
	auto npRez = machines.narrowphaser.RayTrace(*bpRez);//ブロードフェーズ結果からナローフェーズを行う

	cout << "anyhit phase began" << endl;
	auto closestHits = machines.anyhit.Anyhit(*npRez, genhead);//レイの遮蔽を計算しclosest-hitを計算する　ここでは世代内idを使っているので注意

	cout << "shading began" << endl;
	auto rayPayloads=machines.materialer.Shading(*closestHits);//レイの表面での振る舞いを計算 next gen raysを生成
	machines.rooter.AddPayloads(*rayPayloads,genhead);

	cout << "developping began" << endl;
	auto pixels = machines.developper.Develop(*machines.rooter.GetHierarchy());//レイヒエラルキーから現像する

	cout << "It is going to be completly soon..." << endl;
	PrintBmp<CAMERA_RESOLUTION>("out.bmp", *pixels);

	return 0;
}