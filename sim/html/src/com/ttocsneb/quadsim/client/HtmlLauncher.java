package com.ttocsneb.quadsim.client;

import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.backends.gwt.GwtApplication;
import com.badlogic.gdx.backends.gwt.GwtApplicationConfiguration;
import com.badlogic.gdx.backends.gwt.preloader.Preloader.PreloaderCallback;
import com.badlogic.gdx.backends.gwt.preloader.Preloader.PreloaderState;
import com.google.gwt.core.client.GWT;
import com.google.gwt.dom.client.Style;
import com.google.gwt.dom.client.Style.Unit;
import com.google.gwt.user.client.ui.Image;
import com.google.gwt.user.client.ui.InlineHTML;
import com.google.gwt.user.client.ui.Panel;
import com.google.gwt.user.client.ui.SimplePanel;
import com.google.gwt.user.client.ui.VerticalPanel;
import com.ttocsneb.quadsim.Main;

public class HtmlLauncher extends GwtApplication {

		public static final int width = 960, height = 540;
	
        @Override
        public GwtApplicationConfiguration getConfig () {
                return new GwtApplicationConfiguration(width, height);
        }

        @Override
        public ApplicationListener getApplicationListener () {
                return new Main();
        }
        
        @Override
        public PreloaderCallback getPreloaderCallback () {

        	final Panel preloaderPanel = new VerticalPanel();
        	preloaderPanel.setStyleName("gdx-preloader");
        	final Image logo = new Image(GWT.getModuleBaseURL() + "logo.png");
        	logo.setStyleName("logo");
        	preloaderPanel.add(logo);
        	final Panel meterPanel = new SimplePanel();
        	meterPanel.setStyleName("gdx-meter");
        	meterPanel.addStyleName("blue");
        	
        	final InlineHTML meter = new InlineHTML();
        	final Style meterStyle = meter.getElement().getStyle();
        	meterStyle.setWidth(0, Unit.PCT);
        	meterPanel.add(meter);
        	preloaderPanel.add(meterPanel);
        	getRootPanel().add(preloaderPanel);
        	return new PreloaderCallback() {
				
				@Override
				public void update(PreloaderState state) {
					meterStyle.setWidth(100f * state.getProgress(), Unit.PCT);
					
				}
				
				@Override
				public void error(String file) {
					System.out.println("Error: " + file);
				}
			};
        }
}