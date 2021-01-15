#include <HumanIntentionRecognition.h>
HumanIntentionRecognition::HumanIntentionRecognition(int goal_num) {
  std::string dummyline;
  goal_number = goal_num;
  state_probability.resize(goal_number + 2);
  observation.resize(goal_number);
  observation_history = Eigen::VectorXd::Ones(goal_number);
  emission_vector.resize(goal_number + 2);
  Transition_matrix.resize(goal_number + 2, goal_number + 2);
  Transition_matrix << 1 - alpha, 0, 0, alpha, 0,
      0, 1 - alpha, 0, alpha, 0, 0, 0, 1 - alpha, alpha, 0, beta, beta, beta,
      1 - goal_number * beta * gamma, gamma, 0, 0, 0, delta, 1 - delta;
  for (int i = 0; i < goal_number + 2; i++) {
    state_probability(i) = 0;
  }
  state_probability(goal_number) = 1;
  // std::cout << state_probability;
}

void HumanIntentionRecognition::LoadObservation(
    Eigen::VectorXd current_observation, Eigen::MatrixXd dummy_observations) {
  std::cout << "Current Observation: " << current_observation.transpose()
            << '\n';

  auto max_dummy = dummy_observations.colwise().maxCoeff();
  auto min_dummy = dummy_observations.colwise().minCoeff();
  std::cout << "Max Observation: " << max_dummy << '\n';
  std::cout << "Min Observation: " << min_dummy << '\n';

  auto numerator = max_dummy - current_observation.transpose();
  auto denominator = max_dummy - min_dummy;
  for (int i = 0; i < observation.size(); i++) {
    if (denominator(i) > 0.000001) {
      observation(i) = numerator(i) / denominator(i);
    }
  }
  std::string obs_file = "../data/results/observation.csv";

  std::ifstream t(obs_file.c_str());
  std::string current((std::istreambuf_iterator<char>(t)),
                      std::istreambuf_iterator<char>());

  std::ofstream file(obs_file.c_str());
  file << current << observation.transpose().format(CSVFormat) << '\n';

  std::cout << "Observation: " << observation.transpose() << '\n';
}

void HumanIntentionRecognition::UpdateTransition() {
  observation_history =
      ((buffer_size - 1) * observation_history + observation) / buffer_size;
  fi = observation_history.maxCoeff();
  if (fi > 0.5) {
    for (int i = 0; i < goal_number; i++) {
      emission_vector(i) = tanh(observation(i));
    }
    double max_obs = observation.maxCoeff();
    int tmp_idx;
    for (int i = 0; i < goal_number; i++) {
      if (abs(observation(i) - max_obs) < 0.0000000001) {
        observation(i) = 0;
        tmp_idx = i;
      }
    }
    double second_max_obs = observation.maxCoeff();
    observation(tmp_idx) = max_obs;

    double diff = max_obs - second_max_obs;
    emission_vector(goal_number) = tanh(1 - diff);
    emission_vector(goal_number + 1) = 0;
  } else {
    for (int i = 0; i < goal_number; i++) {
      emission_vector(i) = 0;
    }
    emission_vector(goal_number) = tanh(0.1);
    emission_vector(goal_number + 1) = 1 - fi;
  }
  emission_vector /= emission_vector.sum();
  std::cout << "Emission: " << emission_vector.transpose() << '\n';
}

void HumanIntentionRecognition::UpdateProbabilities() {
  // define transition
  Eigen::MatrixXd C;
  C.resize(2, goal_number + 2);
  C *= 0;
  C.row(0) = state_probability;
  // std::cout << C;
  Eigen::VectorXd temp;
  temp.resize(goal_number + 2);
  for (int i = 0; i < goal_number + 2; i++) {
    for (int j = 0; j < goal_number + 2; j++) {
      temp(j) = C(0, j) * Transition_matrix(j, i);
    }
    C(1, i) = temp.maxCoeff() * emission_vector(i);
  }
  state_probability = C.row(1);
  state_probability /= state_probability.sum();
  std::cout << "Probabilities: " << state_probability << '\n';
  std::cout << "Normalized goals: "
            << state_probability.head(3) / state_probability.head(3).sum()
            << '\n';
  std::string prob_file = "../data/results/output.csv";

  std::ifstream t(prob_file.c_str());
  std::string current((std::istreambuf_iterator<char>(t)),
                      std::istreambuf_iterator<char>());

  std::ofstream file(prob_file.c_str());
  file << current << state_probability.format(CSVFormat) << '\n';
}
