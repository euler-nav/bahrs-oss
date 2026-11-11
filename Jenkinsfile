pipeline {
    agent { label 'windows' }

    stages {
        stage('Build STM32 Debug') {
            steps {
                bitbucketStatusNotify(buildState: 'INPROGRESS')
                bat '05_Jenkins/Build_TargetApp_STM32_V2.bat debug'
                bat '05_Jenkins/Build_TargetApp_STM32_V3.bat Debug'
            }
        }
        stage('Build STM32 Release') {
            steps {
                bat '05_Jenkins/Build_TargetApp_STM32_V2.bat release'
                bat '05_Jenkins/Build_TargetApp_STM32_V3.bat Release'
            }
        }
        stage('Build STM32 Debug_WithDebugOutput') {
            steps {
                bat '05_Jenkins/Build_TargetApp_STM32_V2.bat debug-with-debug-output'
                bat '05_Jenkins/Build_TargetApp_STM32_V3.bat debug-with-debug-output'
            }
        }
        stage('Build STM32 DroneCAN variant') {
            steps {
                bat '05_Jenkins/Build_TargetApp_STM32_V2.bat release dronecan'
                bat '05_Jenkins/Build_TargetApp_STM32_V3.bat release dronecan'
            }
        }
        stage('Build VS tools Debug') {
            steps {
                bat '05_Jenkins/Build_VS.bat Debug'
            }
        }
        stage('Build VS tools Release') {
            steps {
                bat '05_Jenkins/Build_VS.bat Release'
            }
        }
        stage('Unit Tests VS Debug') {
            steps {
                bat '05_Jenkins/Unit_Tests.bat Debug'
            }
        }
        stage('Unit Tests VS Release') {
            steps {
                bat '05_Jenkins/Unit_Tests.bat Release'
            }
        }
        stage('Build docs') {
            steps {
                bat '00_Documentation/generate_documentation.bat V2'
                bat '00_Documentation/generate_documentation.bat V3'
            }
        }
    }
    post { 
        success { 
            bitbucketStatusNotify(buildState: 'SUCCESSFUL')
        }
        failure { 
            bitbucketStatusNotify(buildState: 'FAILED')
        }
    }
}