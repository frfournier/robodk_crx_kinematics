function Controller() {
	gui.setSilent(true);

    installer.autoRejectMessageBoxes();
    installer.setMessageBoxAutomaticAnswer("OverwriteTargetDirectory", QMessageBox.Yes);
    installer.setMessageBoxAutomaticAnswer("installationErrorWithRetry", QMessageBox.Ignore);
    installer.installationFinished.connect(function() {
        console.log("install finished");
        gui.clickButton(buttons.NextButton);
    })
}

Controller.prototype.WelcomePageCallback = function() {
    console.log("Welcome");
    gui.clickButton(buttons.NextButton, 3000);
}

Controller.prototype.CredentialsPageCallback = function() {
    console.log("CredentialsPageCallback");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.IntroductionPageCallback = function() {
    console.log("IntroductionPageCallback");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.TargetDirectoryPageCallback = function()
{
    console.log("TargetDirectoryPageCallback");
    var widget = gui.currentPageWidget();
    gui.currentPageWidget().TargetDirectoryLineEdit.setText("/opt/robodk");
    gui.clickButton(buttons.NextButton, 3000);
}

Controller.prototype.ComponentSelectionPageCallback = function() {
    console.log("ComponentSelectionPageCallback");
    var widget = gui.currentPageWidget();
    widget.selectAll();
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.LicenseAgreementPageCallback = function() {
    console.log("LicenseAgreementPageCallback");
    gui.currentPageWidget().AcceptLicenseRadioButton.setChecked(true);
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.StartMenuDirectoryPageCallback = function() {
    console.log("StartMenuDirectoryPageCallback");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.ReadyForInstallationPageCallback = function()
{
    console.log("ReadyForInstallationPageCallback");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.FinishedPageCallback = function() {
    console.log("FinishedPageCallback");
    gui.currentPageWidget().FormFinal.chkOpenRoboDK.setChecked(false);
    gui.clickButton(buttons.FinishButton);
    console.log("Press any key to continue");
}