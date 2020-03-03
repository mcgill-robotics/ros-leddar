pipeline {
  agent any
  environment {
    PACKAGE_NAME = 'leddar'
    ROS_WORKSPACE = "${WORKSPACE}_ws"
    LEDDAR_LIB_DIR = '/usr/lib/leddar'
  }
  stages {
    stage('Setup') {
      steps {
        sh 'printenv'
        sh """
          mkdir -p ${ROS_WORKSPACE}/src
          cp -R . ${ROS_WORKSPACE}/src/${PACKAGE_NAME}
        """
      }
    }
    stage('Build') {
      steps {
        dir(path: "${ROS_WORKSPACE}") {
          sh '''
            . /opt/ros/melodic/setup.sh
            catkin build --no-status --verbose
          '''
        }

      }
    }
    stage('Test') {
      steps {
        dir(path: "${ROS_WORKSPACE}") {
          sh '''
            . /opt/ros/melodic/setup.sh
            . devel/setup.sh
            catkin run_tests
            catkin_test_results build --verbose
          '''
        }
      }
    }
    stage('Sanity Check') {
      parallel {
        stage('Lint') {
          steps {
            sh '''
              . /opt/ros/melodic/setup.sh
              catkin lint --explain -W2 --strict . \
                --ignore env_var --ignore link_directory
            '''
          }
        }
        stage('Format') {
          steps {
            sh '''
              FILES="$(find ${PWD} \
                -iname '*.h' -o \
                -iname '*.c' -o \
                -iname '*.cpp' -o \
                -iname '*.hpp'
              )"
              CHANGES=""
              for f in ${FILES}; do
                CURR_CHANGES="$(clang-format ${f} | diff ${f} -)"
                CHANGES="${CHANGES}${CURR_CHANGES}"
              done
              [ -z "${CHANGES}" ]
            '''
          }
        }
      }
    }
  }
  post {
    always {
      dir(path: "${ROS_WORKSPACE}") {
        archiveArtifacts(artifacts: "logs/**/*.log", fingerprint: true)
        script {
          def files = findFiles glob: 'build/**/test_results/**/*.xml'
          if (files.length > 0) {
            junit "build/**/test_results/**/*.xml"
          }
        }
      }
      sh "rm -rf ${ROS_WORKSPACE}"
    }
  }
}
