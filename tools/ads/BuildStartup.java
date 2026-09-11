package local.ads;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.charset.StandardCharsets;
import org.eclipse.ui.IStartup;
import org.eclipse.ui.PlatformUI;
import org.eclipse.core.resources.*;
import org.eclipse.core.runtime.*;
import org.eclipse.core.runtime.jobs.Job;
import org.eclipse.cdt.managedbuilder.core.*;

/** Runs inside the real ADS workbench; never invokes TASKING standalone. */
public final class BuildStartup implements IStartup {
    public void earlyStartup() {
        Job job = new Job("Automated ADS project build") {
            protected IStatus run(IProgressMonitor monitor) {
                int errors = 0;
                StringBuilder report = new StringBuilder();
                try {
                    IWorkspace workspace = ResourcesPlugin.getWorkspace();
                    IWorkspaceDescription wd = workspace.getDescription();
                    wd.setAutoBuilding(false);
                    workspace.setDescription(wd);
                    Path root = Path.of(System.getProperty("local.ads.project"));
                    IProjectDescription pd = workspace.loadProjectDescription(new org.eclipse.core.runtime.Path(root.resolve(".project").toString()));
                    pd.setLocation(new org.eclipse.core.runtime.Path(root.toString()));
                    IProject project = workspace.getRoot().getProject(pd.getName());
                    if (!project.exists()) project.create(pd, monitor);
                    if (!project.isOpen()) project.open(monitor);
                    project.refreshLocal(IResource.DEPTH_INFINITE, monitor);
                    IManagedBuildInfo info = ManagedBuildManager.getBuildInfo(project);
                    IConfiguration selected = null;
                    for (IConfiguration c : info.getManagedProject().getConfigurations())
                        if (c.getName().equals("Debug")) selected = c;
                    if (selected == null) throw new IllegalStateException("Debug configuration missing");
                    ManagedBuildManager.setDefaultConfiguration(project, selected);
                    ManagedBuildManager.saveBuildInfo(project, true);
                    if (Boolean.getBoolean("local.ads.clean")) project.build(IncrementalProjectBuilder.CLEAN_BUILD, monitor);
                    project.build(IncrementalProjectBuilder.INCREMENTAL_BUILD, monitor);
                    Thread.sleep(1000);
                    PlatformUI.getWorkbench().getDisplay().syncExec(() -> {
                        try {
                            String console = org.eclipse.cdt.ui.CUIPlugin.getDefault()
                                .getConsoleManager().getConsoleDocument(project).get();
                            Files.writeString(Path.of(System.getProperty("local.ads.result")).resolveSibling("build.log"),
                                console, StandardCharsets.UTF_8);
                        } catch (Exception e) { throw new RuntimeException(e); }
                    });
                    for (IMarker m : project.findMarkers(IMarker.PROBLEM, true, IResource.DEPTH_INFINITE)) {
                        int severity = m.getAttribute(IMarker.SEVERITY, -1);
                        if (severity == IMarker.SEVERITY_ERROR) errors++;
                        if (severity >= IMarker.SEVERITY_WARNING)
                            report.append(severity == IMarker.SEVERITY_ERROR ? "ERROR " : "WARNING ")
                                .append(m.getResource().getProjectRelativePath()).append(":")
                                .append(m.getAttribute(IMarker.LINE_NUMBER, 0)).append(" ")
                                .append(m.getAttribute(IMarker.MESSAGE, "")).append("\n");
                    }
                    report.append("ERRORS=").append(errors).append("\n");
                } catch (Throwable e) {
                    errors++;
                    java.io.StringWriter sw = new java.io.StringWriter();
                    e.printStackTrace(new java.io.PrintWriter(sw));
                    report.append(sw);
                }
                try {
                    Files.writeString(Path.of(System.getProperty("local.ads.result")),
                        "STATUS=" + (errors == 0 ? "OK" : "FAILED") + "\n" + report,
                        StandardCharsets.UTF_8);
                } catch (Exception e) { e.printStackTrace(); }
                PlatformUI.getWorkbench().getDisplay().asyncExec(() -> PlatformUI.getWorkbench().close());
                return Status.OK_STATUS;
            }
        };
        job.schedule(2000);
    }
}
