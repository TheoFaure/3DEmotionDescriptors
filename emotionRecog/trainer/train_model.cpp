#include "../common.hpp"
#include <sys/stat.h>
#include <unistd.h>

int read_file(int emotion_index, int file_nb, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::string type(get_emotion_from_index(emotion_index));
  std::string file(SSTR(file_nb));
  std::string file_name = ("/home/theo/Documents/3D/Projet/emotionRecog/data/images/" + type + "/" + file + ".pcd").c_str();
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }
}

void write_histogram_on_file(int emotion_index, int file_nb, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
	pcl::VFHSignature308 descriptor = vfhs->points[0];
	
	std::string type(get_emotion_from_index(emotion_index));
  std::string file(SSTR(file_nb));
//	std::string out_file_name = ("/home/theo/Documents/3D/Projet/emotionRecog/data/histograms/" + type + "/" + file + ".txt").c_str();
	std::ofstream out_file(("/home/theo/Documents/3D/Projet/emotionRecog/data/histograms/" + type + "/" + file + ".txt").c_str());
  if (out_file.is_open())
  {
		out_file << descriptor;
		out_file.close();
	}
}

void add_histogram(cv::Mat * histograms, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
{
	pcl::VFHSignature308 descriptor = vfhs->points[0]; //The histogram
	
	std::ostringstream stream;
	stream << descriptor;
	std::string histogram_str = stream.str();
	
	histogram_str = histogram_str.substr(1, histogram_str.size() - 2);
	std::vector<std::string> histogram_vector = split(histogram_str, ',');
	cv::Mat histogram = cv::Mat::zeros(1, 308, CV_32FC1);
	for (int j = 0 ; j < 308 ; j++)
	{
		std::istringstream buffer(histogram_vector[j]);
		int temp;
		buffer >> temp;
		histogram.at<float>(0, j) = temp;
		//histogram.at<float>(0, j) = std::stoi(histogram_vector[j]);
	}
	
	histograms->push_back(histogram);
}

bool file_exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

int process_images_emotion(int emotion_index, int nb_images,cv::Mat * histograms)
{
	for (int i = 1 ; i <= nb_images ; i++)
	{
		//Preprocessing
		std::string type(get_emotion_from_index(emotion_index));
		std::string file(SSTR(i));
		if (!file_exists(("/home/theo/Documents/3D/Projet/emotionRecog/data/histograms/" + type + "/" + file + ".txt").c_str()))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
			if (read_file(emotion_index, i, cloud) == -1)
			{ return -1; }
		
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
			filter_file(cloud, cloud_filtered);
		
			//Compute histograms
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
			compute_normals(cloud_filtered, cloud_normals);
		
			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
			compute_VFH(cloud_filtered, cloud_normals, vfhs);
		
			write_histogram_on_file(emotion_index, i, vfhs);
			add_histogram(histograms, vfhs);
		} else
		{
			std::string line;
			std::ifstream myfile (SSTR("/home/theo/Documents/3D/Projet/emotionRecog/data/histograms/" + type + "/" + file + ".txt").c_str());
			if (myfile.is_open())
			{
				if ( std::getline (myfile,line) )
				{
					std::string histogram_str = line.substr(1, histogram_str.size() - 2);
					std::vector<std::string> histogram_vector = split(histogram_str, ',');
					cv::Mat histogram = cv::Mat::zeros(1, 308, CV_32FC1);
					for (int j = 0 ; j < 308 ; j++)
					{
						std::istringstream buffer(histogram_vector[j]);
						int temp;
						buffer >> temp;
						histogram.at<float>(0, j) = temp;
					}
					std::cout << "Data in the file: " << histogram.size[0] << " " << histogram.size[1] << endl;
					histograms->push_back(histogram);
				}
				myfile.close();
			} else
			{
				return -1;
			}
		}
	}
	return 0;
}

cv::Mat create_labels(std::vector<int> nb_images)
{
	int nb_total = nb_images[0] + nb_images[1] + nb_images[2] + nb_images[3] + nb_images[4];
	cv::Mat labels = cv::Mat::zeros(nb_total, 1, CV_32SC1);
	//Tristeza = 0, Allegria = 1, Miedo = 2, Sorpresa = 3, Colera = 4
  for (int i = 0 ; i < nb_total ; i++)
  {
		if (i < nb_images[0])
			labels.at<float>(0, i) = 0;
		else if (i < nb_images[1])
			labels.at<float>(0, i) = 1;
		else if (i < nb_images[2])
			labels.at<float>(0, i) = 2;
		else if (i < nb_images[3])
			labels.at<float>(0, i) = 3;
		else
			labels.at<float>(0, i) = 4;
  }
  
  return labels;
}

void create_RTree(cv::Mat* histograms, cv::Mat* labels)
{
	cv::Ptr<cv::ml::RTrees> rtree = cv::ml::RTrees::create();
	rtree->train( *histograms , cv::ml::ROW_SAMPLE , *labels );
	rtree->save("../model_RTree");
}

void create_SVM(cv::Mat* histograms, cv::Mat* labels, int kfold)
{
	// Set up SVM's parameters
	
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	svm->setType(cv::ml::SVM::C_SVC);
	svm->setKernel(cv::ml::SVM::LINEAR);
	svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));

	std::cout << "Data in the hisogram: " << histograms->size[0] << " " << histograms->size[1] << endl;
	
	// Train the SVM
  cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(*histograms, cv::ml::ROW_SAMPLE, *labels);
	svm->train(td);

/*
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	svm->setType(cv::ml::SVM::C_SVC);
	svm->setKernel(cv::ml::SVM::POLY);
	svm->setDegree(3);
	
	cv::Ptr<cv::ml::TrainData> train_data = cv::ml::TrainData::create(*histograms, cv::ml::ROW_SAMPLE, *labels);
	
	svm->trainAuto(train_data, kfold, cv::ml::SVM::getDefaultGrid(cv::ml::SVM::C), cv::ml::SVM::getDefaultGrid(cv::ml::SVM::GAMMA), cv::ml::SVM::getDefaultGrid(cv::ml::SVM::P), cv::ml::SVM::getDefaultGrid(cv::ml::SVM::NU), cv::ml::SVM::getDefaultGrid(cv::ml::SVM::COEF), cv::ml::SVM::getDefaultGrid(cv::ml::SVM::DEGREE), false);
	*/
	svm->save("../model_SVM");
}

int
 main (int argc, char** argv)
{
//DATA PROCESSING
	std::vector<int> nb_images = get_number_images();
	std::cout << "Number of images:" << std::endl;
	std::cout << nb_images[0] << " " << nb_images[1] << " " << nb_images[2] << " " << nb_images[3] << " " << nb_images[4] << " " << std::endl;
	cv::Mat histograms = cv::Mat::zeros(0, 308, CV_32FC1);
	// for each emotion, process the images.
	
	for (int i = 0 ; i < nb_images.size() ; i++)
	{
		std::cout << "Processing emotion " << i << "..." << std::endl;
		if (process_images_emotion(i, nb_images[i], &histograms) == -1)
		{ 
			std::cout << "Uuuuch, bug!" << std::endl;
			return (-1);
		}	
	}
	std::cout << "All emotions processed." << std::endl;
	
	cv::Mat labels = create_labels(nb_images);
	
//MODEL CREATION
	
	if (argc >= 2)
	{
		std::string arg1 = argv[1];
		if (arg1.compare("RTree") == 0)
		{
			std::cout << "Creating RTree model." << std::endl;
			create_RTree(&histograms, &labels);
		} else 
		{
			std::istringstream buffer(argv[2]);
			int kfold;
			buffer >> kfold;
			std::cout << "Creating SVM model." << std::endl;
			create_SVM(&histograms, &labels, kfold);
		}
	} else
	{
		std::cout << "Creating SVM model." << std::endl;
		create_SVM(&histograms, &labels, 10);
	}
  return (0);
}
