#include"general.hpp"
#include"sub.hpp"
#include "int24.hpp"

using namespace std;
using namespace Eigen;
//using namespace half_float::literal;


using evec3 = Eigen::Vector3<halff>;
using Affine3h = Eigen::Transform<halff, 3, 2>;

constexpr size_t MAX_GENERATIONS = 20;

constexpr size_t CORE_NUM = 1;
constexpr size_t CAMERA_RESOLUTION = 768;
const halff CAMERA_FOV = halff(60. * std::numbers::pi / 180.0);
const extern sindex RAYNUM_LIMIT_BRUNCH = 1;//一本のレイから生じる分岐の最大値
constexpr exindex RAYNUM_LIMIT_GENERATION = (CAMERA_RESOLUTION * CAMERA_RESOLUTION);//一世代のレイの最大数

const extern exindex RAYNUM_LIMIT_ALL = RAYNUM_LIMIT_GENERATION * MAX_GENERATIONS*2;//全世代のレイの合計の最大数
constexpr exindex RAYNUM_LIMIT_TERMINATES = RAYNUM_LIMIT_GENERATION*2;//終端レイの最大数

constexpr size_t BLASNUM_LIMIT = 8;//blasの数の制限　これだけのオブジェクトを配置できる
const extern size_t ATTRIBUTE_SIZE = 3;//アトリビュートの大きさ[word]
constexpr size_t TRIANGLES_NUM_LIMIT = 2048*32;//triangleの最大数 これだけのtriangleを用意できる


const halff IGNORE_NEARHIT = 0.01;//レイの当たり判定はt=これ以降で発生する　透過光を通過させるときに必要
const halff IGNORE_PARALLELHIT = cos(89.99/180.*std::numbers::pi);//norm dot directionがこれ以下のときナローフェーズが発生　カリングと並行光の無視が発生する

const halff	TRIANGLE_EXTEND_SIZE = 32.0;//ポリゴンのサイズをイプシロンのこれ倍だけ拡張する 大きいほどナローフェーズが甘くなる
const halff AABB_TIMES_MARGINE = 0.0;//vsAABBの交差時間のマージン　大きいほどブロードフェーズが甘くなる



#include "shaders.cpp"
const std::vector<std::tuple<string, hmat4,toolkit::materializer<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH,toolkit::attributeFramework<ATTRIBUTE_SIZE>>::shader>> model_gen = {
	//std::make_tuple("../monkey.dae",DiagonalMatrix<halff,4>(.3,.3,.3,1.)*Affine3h(Translation<halff,3>(evec3(-0.0,0.0,0.0))).matrix(),GetColorShader<RAYNUM_LIMIT_ALL, RAYNUM_LIMIT_BRUNCH, ATTRIBUTE_SIZE>(hvec3(1., 0., 0.))),
	std::make_tuple("../minicube.dae",Eigen::Matrix4d::Identity(),HitNormColor)//,
	//std::make_tuple("../wave.dae",Affine3h(Translation<halff,3>(evec3(0.0,-3.0,0.0))).matrix(),HitMirror)
};
/*
tlasをアウターからなんとか構築し　それにレイトレース処理を行うことでrayHierarchyに変換　それを現像処理することでフレームを作成する
アウターからコマンドを使ってカメラとblasとそれぞれの変換を送信　tlasを構成する
*/


//手順0(pre-phase)  アウターで行われるデータ構造の準備
struct prephaseRez {
	std::vector<sptr<blas>> objs;
	sptr<camera> cam;
};
prephaseRez PrePhase() {
	//SetMaxWH(CAMERA_RESOLUTION, CAMERA_RESOLUTION);//bmpに解像度を設定
	prephaseRez rez;
	//手順0(事前処理) モデルを読み込んでblasを作成する&カメラを作成する
	size_t trianglesnum_sum = 0;
	for (const auto& p : model_gen) {
		dmod tempmod;
		ModLoader(std::get<0>(p), tempmod);
		trianglesnum_sum += tempmod.size();
		sptr<blas> obj(new blas(tempmod));
		rez.objs.push_back(obj);
	}
	if (trianglesnum_sum > TRIANGLES_NUM_LIMIT)throw std::out_of_range("Triangle included LAS are more than TRIANGLES_NUM_LIMIT");

	//仮のアトリビュート
	/*for (size_t i=0;i<rez.objs.size();i++)
		MakeDammyAttributes<toolkit::attributeFramework<ATTRIBUTE_SIZE>>(rez.objs.at(i).get()->triangles.size(),machines->memory.GetAttributes()->at(i));*/

	//カメラを生成
	rez.cam.reset(new camera(CAMERA_RESOLUTION, CAMERA_RESOLUTION, -camera::CalcDistFromFov(CAMERA_FOV)));

	return rez;
}

int main() {

	//つぎにヘッダを読み込む
	const std::string dicheaderPrefix = R"(C:\local\user\lensball\lensball\rez\SimVis\dic\dic)";
	projRefraDicHeader dicheader;
	{
		ifstream headerIfs(dicheaderPrefix+".head", std::ios::binary);
		cereal::BinaryInputArchive i_arc(headerIfs);
		i_arc(dicheader);
	}

	//結果のフレームはここに保存する
	const std::string frameSavePrefix = R"(C:\local\user\lensball\lensball\rez\SimVis\frames\frame)";

	constexpr size_t rayTraceThreadNum = 18;
	std::array<uptr<std::thread>, rayTraceThreadNum> rayTraThreads;
	std::array<bool, rayTraceThreadNum> rayTraIsFin;
	const auto rayTracePipeline = [&](const size_t sceneid, decltype(rayTraIsFin)::iterator isfin) {
		//シーンごとにレイトレーシングを行う必要がある　つまりレイリストが手に入る
		std::list<arrow3> raylist;
		{
			ifstream headerIfs(dicheaderPrefix + ".part" + to_string(sceneid), std::ios::binary);
			cereal::BinaryInputArchive i_arc(headerIfs);
			i_arc(raylist);
		}

		//これをカメラにする
		auto raylistite = raylist.cbegin();
		sptr<camera> cam = make_shared<camera>();//行ごとにデータが入っていくのに気をつけて
		std::vector<std::pair<size_t, size_t>> pixPoses;
		exindex rayIdCount = 0;
		for (size_t vd = 0; vd < dicheader.verticalRes; vd++) {
			for (size_t hd = 0; hd < dicheader.horizontalRes; hd++, raylistite++) {
				//ゼロベクトルが来ることもある
				if ((*raylistite).dir() == uvec<3>::Zero()) {
					//来た場合評価する必要がないってこと
					continue;
				}

				cam->push_back(ray());
				pixPoses.push_back(make_pair(hd, vd));

				auto& target = cam->back();
				target.org() = hvec3((*raylistite).org().x(), (*raylistite).org().y(), (*raylistite).org().z());
				target.way() = hvec3(-(*raylistite).dir().x(), -(*raylistite).dir().y(), -(*raylistite).dir().z());
				target.indexed(rayIdCount++);//indexは座標を元に決めれそう
			}
		}
		//モデルを用意
		size_t trianglesnum_sum = 0;
		std::list<sptr<blas>> lases;
		uptr<las> world = make_unique<las>();
		for (const auto& p : model_gen) {
			dmod tempmod;
			ModLoader(std::get<0>(p), tempmod);
			trianglesnum_sum += tempmod.size();
			sptr<blas> obj(new blas(tempmod));
			world->push_back(make_pair(std::get<1>(p), obj));
		}
		if (trianglesnum_sum > TRIANGLES_NUM_LIMIT)throw std::out_of_range("Triangle included LAS are more than TRIANGLES_NUM_LIMIT");

		//ブロードフェーズ
		toolkit::broadphaser<1> bper(AABB_TIMES_MARGINE);
		const auto bpResult = bper.Broadphase(*cam, world.get());
		//ナローフェーズ
		toolkit::narrowphaser nper(TRIANGLE_EXTEND_SIZE);
		const auto npResult = nper.RayTrace(*bpResult, IGNORE_NEARHIT, IGNORE_PARALLELHIT, world.get());

		toolkit::obstructer<RAYNUM_LIMIT_GENERATION> obser;
		std::unordered_map<exindex, closesthit> closests;
		obser.Anyhit(*npResult, 0, closests);

		//辞書ファイルをロードして方向をマッピングする
		sptr<bitmapx> colors = make_shared<bitmapx>(dicheader.horizontalRes * dicheader.verticalRes);
		for (auto& c : *colors)
			c = hvec3({ 1.,1.,1. });//表示されなかったら
		exindex count = 0;
		for (const auto& i : *cam) {
			const auto thispos = pixPoses.at(count);
			auto ite = closests.find(count);
			colors->at(thispos.first + thispos.second * dicheader.horizontalRes) = { 0.,0.,ite != closests.end() ? 1. : 0. };

			count++;
		}

		PrintBmpWithAnotherSize_YOU_MUST_READ_COMMENT(frameSavePrefix+to_string(sceneid)+".bmp", *colors,dicheader.horizontalRes,dicheader.verticalRes);

		//結果を作成
		cout << "developped" << endl;
		//dever.Develop()
		//auto pixels = machines->developer.Develop(machines->memory.GetPayloadsAllgen(), machines->memory.GetTerminatesGen());//レイヒエラルキーから現像する

		//cout << "It is going to be completly soon..." << endl;
		//PrintBmpWithAnotherSize_YOU_MUST_READ_COMMENT<CAMERA_RESOLUTION>(frameSavePrefix+to_string(sceneid)+".bmp", *pixels,dicheader.horizontalRes,dicheader.verticalRes);

		*isfin = true;
	};
	//rayTracePipeline(0, rayTraIsFin.begin());

	//複数スレッドに回転角度を変えながら割り当てる
	for (size_t rdgen = 0; rdgen < dicheader.rotationRes; rdgen++) {
		std::cout << "scene: " << rdgen << endl;
		//このrdgenでの処理を開いているスレッドに割り付けたい
		bool isfound = false;
		while (!isfound) {//割り付けられなければ繰り返す
			for (size_t th = 0; th < rayTraceThreadNum; th++)
				if (!rayTraThreads.at(th)) {//空きなら割付
					if (!isfound) {//一つのインデックスには一回だけ割り付ける
						isfound = true;
						rayTraIsFin.at(th) = false;//フラグをクリアして

						rayTraThreads.at(th).reset(new std::thread(rayTracePipeline, rdgen, rayTraIsFin.begin() + th));//スレッド実行開始
					}
				}
				else if (rayTraIsFin.at(th)) {//空いてなくて終わってるなら
					rayTraThreads.at(th).get()->join();
					rayTraThreads.at(th).release();//リソースを開放
				}
		}
	}

	//全スレッドの終了を待つ
	for (size_t thd = 0; thd < rayTraceThreadNum;thd++) {
		if (rayTraThreads.at(thd)) {
			rayTraThreads.at(thd)->join();
			rayTraThreads.at(thd).release();
		}
	}

	return 0;
}