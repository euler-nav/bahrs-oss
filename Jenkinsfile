pipeline {
    agent { label 'windows' }

    stages {
        stage('Build STM32 Debug') {
            steps {
                bitbucketStatusNotify(buildState: 'INPROGRESS')
                bat '05_Jenkins/Build_TargetApp_STM32.bat Debug'
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